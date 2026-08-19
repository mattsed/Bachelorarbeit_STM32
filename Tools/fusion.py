"""
Sensorfusion (offline, rein numpy -- keine Zusatzpakete):

1. Lagefilter (Madgwick-Komplementaerfilter, IMU-Variante ohne Magnetometer)
   Verschmilzt Gyro (schnell, aber driftend) und Accelerometer (driftfrei,
   aber von Bewegungen gestoert) zu einer Orientierungsschaetzung als
   Quaternion. Daraus wird die Richtung der Erdbeschleunigung im
   Sensorrahmen bestimmt und aus dem Messsignal herausgerechnet --
   Ergebnis ist die NEIGUNGSBEREINIGTE Laengsbeschleunigung: bei 20 %
   Gefaelle steckt sonst allein ~0,34 g Hangabtrieb im Rohsignal, was
   jede Verzoegerungsmessung verfaelschen wuerde.

   Grenze (physikalisch, keine Schwaeche der Implementierung): ohne
   Magnetometer ist nur Roll und Nick beobachtbar, der Kurswinkel (Yaw)
   driftet frei. Deshalb wird bewusst NICHT in ein erdfestes Nord/Ost-
   System transformiert, sondern nur die Schwerkraft entlang der
   Fahrzeugachsen kompensiert -- dafuer wird Yaw nicht gebraucht.

2. Geschwindigkeits-Kalman-Filter (1D)
   Zustand ist die Fahrgeschwindigkeit. Praediktion mit 50 Hz aus der
   neigungsbereinigten Laengsbeschleunigung (Schritt 1), Korrektur mit
   1 Hz aus den echten GNSS-Geschwindigkeitsupdates. Ergebnis ist ein
   glattes 50-Hz-Geschwindigkeitsprofil, das auch WAEHREND einer kurzen
   Bremsung den Verlauf zeigt -- die GNSS-Rohdaten liefern dort nur
   1-2 Stuetzstellen.

Beide Filter sind bewusst von Hand implementiert (statt filterpy/ahrs),
damit in der Arbeit jede Gleichung explizit hergeleitet und diskutiert
werden kann; bei 1x1-Kovarianzen ist das problemlos moeglich.

WICHTIG: Die Zuordnung, welche IMU-Achse in Fahrtrichtung zeigt
(FAHRT_ACHSE/FAHRT_VORZEICHEN in konstanten.py), ist noch nicht
kalibriert -- das geht erst mit einer echten Fahrmessung. Bis dahin sind
die Absolutwerte von a_laengs mit Vorsicht zu geniessen; die Architektur
und das Kalman-Verhalten lassen sich aber bereits validieren.
"""

import numpy as np
import pandas as pd

from konstanten import (
    MADGWICK_BETA, FAHRT_ACHSE, FAHRT_VORZEICHEN, FAHRT_SKALIERUNG,
    KALMAN_SIGMA_A_M_S2, KALMAN_SIGMA_GNSS_M_S, G_M_S2,
)


# ------------------------------------------------------------ Lagefilter

def _quaternion_aus_ruhelage(ax: float, ay: float, az: float) -> np.ndarray:
    """Startorientierung aus einer Ruhemessung des Accelerometers.

    Im Stand zeigt der Beschleunigungsvektor exakt entgegen der
    Schwerkraft; daraus folgen Roll- und Nickwinkel direkt (Yaw ist ohne
    Magnetometer nicht bestimmbar und wird 0 gesetzt). Der Filter startet
    so bereits in der richtigen Lage, statt sich erst ueber Sekunden
    einschwingen zu muessen."""
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    # Euler (Yaw=0, Pitch, Roll) -> Quaternion, ZYX-Konvention.
    return np.array([cp * cr, cp * sr, sp * cr, -sp * sr])


def _madgwick_schritt(q: np.ndarray, gx: float, gy: float, gz: float,
                      ax: float, ay: float, az: float,
                      beta: float, dt: float) -> np.ndarray:
    """Ein Madgwick-Update: Gyro integrieren, mit dem Accelerometer per
    Gradientenschritt nachkorrigieren.

    Der Gradientenschritt zieht die Quaternion in die Lage, in der die
    vorhergesagte Schwerkraftrichtung mit der gemessenen uebereinstimmt;
    beta begrenzt die Schrittweite (Vertrauen ins Accelerometer).
    Gyro in rad/s, Accelerometer beliebig skaliert (wird normiert)."""
    q0, q1, q2, q3 = q

    # Aenderungsrate der Quaternion aus der Drehrate (Strapdown-Integration).
    q_dot = 0.5 * np.array([
        -q1 * gx - q2 * gy - q3 * gz,
        q0 * gx + q2 * gz - q3 * gy,
        q0 * gy - q1 * gz + q3 * gx,
        q0 * gz + q1 * gy - q2 * gx,
    ])

    norm_a = np.sqrt(ax * ax + ay * ay + az * az)
    if norm_a > 1e-9:
        ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a
        # Fehlerfunktion: vorhergesagte minus gemessene Schwerkraftrichtung.
        f1 = 2.0 * (q1 * q3 - q0 * q2) - ax
        f2 = 2.0 * (q0 * q1 + q2 * q3) - ay
        f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az
        # Gradient (Jacobi-transponiert mal Fehler), Herleitung s. Madgwick 2010.
        s = np.array([
            -2.0 * q2 * f1 + 2.0 * q1 * f2,
            2.0 * q3 * f1 + 2.0 * q0 * f2 - 4.0 * q1 * f3,
            -2.0 * q0 * f1 + 2.0 * q3 * f2 - 4.0 * q2 * f3,
            2.0 * q1 * f1 + 2.0 * q2 * f2,
        ])
        norm_s = np.linalg.norm(s)
        if norm_s > 1e-9:
            q_dot -= beta * s / norm_s

    q = q + q_dot * dt
    return q / np.linalg.norm(q)


def lagefilter(df: pd.DataFrame, beta: float = MADGWICK_BETA) -> pd.DataFrame:
    """Laeuft ueber alle 50-Hz-Zeilen und ergaenzt drei Spalten:

        roll_deg, pitch_deg   Fahrzeuglage (Waelzen/Nicken) [Grad]
        a_laengs_ms2          neigungsbereinigte Laengsbeschleunigung
                              [m/s^2], Schwerkraftanteil entfernt,
                              Vorzeichen: + = beschleunigen (sofern
                              FAHRT_ACHSE/FAHRT_VORZEICHEN stimmen)

    Der Filter arbeitet zeilenweise mit dem echten Zeitabstand aus t_s
    (nicht mit fest 20 ms), damit auch Logs mit Aussetzern korrekt
    integriert werden."""
    t = df["t_s"].to_numpy()
    acc = df[["imu_ax_g", "imu_ay_g", "imu_az_g"]].to_numpy()
    gyr = np.radians(df[["imu_gx_dps", "imu_gy_dps", "imu_gz_dps"]].to_numpy())

    # Startlage aus dem Mittel der ersten Sekunde (Logger startet im Stand;
    # dieselbe Annahme nutzt schon die Gyro-Bias-Messung der Firmware).
    n0 = min(50, len(df))
    a0 = acc[:n0].mean(axis=0)
    q = _quaternion_aus_ruhelage(a0[0], a0[1], a0[2])

    n = len(df)
    roll = np.empty(n)
    pitch = np.empty(n)
    a_lin = np.empty((n, 3))

    for i in range(n):
        dt = t[i] - t[i - 1] if i > 0 else 0.0
        if dt > 0:
            q = _madgwick_schritt(q, gyr[i, 0], gyr[i, 1], gyr[i, 2],
                                  acc[i, 0], acc[i, 1], acc[i, 2], beta, dt)
        q0, q1, q2, q3 = q
        # Schwerkraftrichtung im Sensorrahmen (3. Zeile der Drehmatrix) --
        # exakt der Vektor, den ein ruhendes Accelerometer messen wuerde.
        gvec = np.array([2.0 * (q1 * q3 - q0 * q2),
                         2.0 * (q0 * q1 + q2 * q3),
                         q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3])
        # Lineare (Bewegungs-)Beschleunigung = Messung minus Schwerkraft [g].
        a_lin[i] = acc[i] - gvec
        roll[i] = np.degrees(np.arctan2(gvec[1], gvec[2]))
        pitch[i] = np.degrees(np.arcsin(np.clip(-gvec[0], -1.0, 1.0)))

    df["roll_deg"] = roll
    df["pitch_deg"] = pitch

    # Alle drei Achsen der neigungsbereinigten Beschleunigung ablegen, nicht
    # nur die ausgewaehlte: kalibrierung.py braucht sie, um FAHRT_ACHSE und
    # FAHRT_VORZEICHEN gegen die GNSS-Geschwindigkeit zu bestimmen.
    for name, spalte in (("x", 0), ("y", 1), ("z", 2)):
        df[f"a_lin_{name}_ms2"] = a_lin[:, spalte] * G_M_S2

    achse = {"x": 0, "y": 1, "z": 2}[FAHRT_ACHSE]
    # FAHRT_SKALIERUNG gleicht die Einbaulage aus: Die Achse sieht nur
    # einen Teil der wahren Laengsbeschleunigung (Pitch rund -62 Grad).
    df["a_laengs_ms2"] = (FAHRT_VORZEICHEN * FAHRT_SKALIERUNG
                          * a_lin[:, achse] * G_M_S2)

    print(f"Fusion Lage:       Roll {roll.mean():+.1f} deg / "
          f"Pitch {pitch.mean():+.1f} deg im Mittel, "
          f"a_laengs {df['a_laengs_ms2'].abs().max():.2f} m/s^2 max "
          f"(Achse {FAHRT_ACHSE}, Massstab {FAHRT_SKALIERUNG:.2f})")
    return df


# ------------------------------------------- Geschwindigkeits-Kalman (1D)

def geschwindigkeitsfilter(df: pd.DataFrame, spur) -> pd.DataFrame:
    """Fusioniert IMU-Laengsbeschleunigung (50 Hz) mit den echten
    GNSS-Geschwindigkeitsupdates (1 Hz) zu einer glatten Geschwindigkeit.

    Klassischer Kalman-Zyklus mit skalarem Zustand v [m/s]:
      Praediktion (jede Zeile):  v <- v + a_laengs*dt,  P <- P + Q(dt)
      Korrektur (je GNSS-Update): K = P/(P+R)
                                  v <- v + K*(v_gnss - v),  P <- (1-K)*P
    P ist die Varianz der Schaetzung, Q das Prozessrauschen (wie sehr die
    IMU-Praediktion pro Schritt danebenliegen kann), R das Messrauschen
    der GNSS-Geschwindigkeit. K ergibt sich daraus automatisch: praezise
    Praediktion -> kleines K (GNSS wird nur leicht eingeblendet), grosse
    Unsicherheit -> grosses K (GNSS zieht die Schaetzung stark zurueck).

    Ergaenzt die Spalte v_fusion_kmh. Ohne GNSS-Spur oder ohne vorherigen
    lagefilter()-Lauf wird nichts veraendert (Hinweis auf der Konsole)."""
    if spur is None or "a_laengs_ms2" not in df.columns:
        print("Fusion v:          uebersprungen (keine GNSS-Spur oder kein Lagefilter)")
        return df

    # Nur ECHTE GNSS-Updates als Messungen verwenden -- die interpolierten
    # Ersatzpunkte aus gnss.py sind keine neue Information.
    echt = spur[~spur["interp"]]
    mess_t = echt["t_s"].to_numpy()
    mess_v = echt["v_km_h"].to_numpy() / 3.6

    t = df["t_s"].to_numpy()
    a = df["a_laengs_ms2"].to_numpy()
    n = len(df)

    r = KALMAN_SIGMA_GNSS_M_S ** 2
    v = float(mess_v[0])     # Start beim ersten GNSS-Wert
    p = r                    # Startunsicherheit = Messunsicherheit
    naechste_messung = 0
    v_out = np.empty(n)

    for i in range(n):
        dt = t[i] - t[i - 1] if i > 0 else 0.0
        if dt > 0:
            # Praediktion: Beschleunigung aufintegrieren; Unsicherheit
            # waechst mit jedem Schritt um das Prozessrauschen.
            v += a[i] * dt
            p += (KALMAN_SIGMA_A_M_S2 * dt) ** 2
        # Alle inzwischen faelligen GNSS-Messungen einarbeiten.
        while naechste_messung < len(mess_t) and mess_t[naechste_messung] <= t[i]:
            k = p / (p + r)
            v += k * (mess_v[naechste_messung] - v)
            p *= (1.0 - k)
            naechste_messung += 1
        # Geschwindigkeit ist ein Betrag (GNSS liefert keinen Rueckwaerts-
        # begriff); negatives Durchintegrieren waere ein Achsen-/Biasfehler.
        v = max(v, 0.0)
        v_out[i] = v

    df["v_fusion_kmh"] = v_out * 3.6
    print(f"Fusion v:          {len(mess_t)} GNSS-Stuetzstellen, "
          f"max {df['v_fusion_kmh'].max():.1f} km/h "
          f"(GNSS-Rohwert max {df['v_km_h'].max():.1f} km/h)")
    return df
