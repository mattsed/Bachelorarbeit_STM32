"""
Automatische Erkennung von Bremsereignissen aus dem Druckverlauf.

Ein Ereignis beginnt, sobald einer der beiden Kanaele (vorne/hinten) die
Startschwelle ueberschreitet, und endet erst, wenn der Druck wieder unter
die -- niedrigere -- Endschwelle faellt (Hysterese: verhindert, dass ein
um die Schwelle flatternder Druck viele Mini-Ereignisse erzeugt). Zu kurze
Ereignisse gelten als Rauschspitzen und werden verworfen.
"""

import pandas as pd

from konstanten import BREMS_START_BAR, BREMS_ENDE_BAR, BREMS_MIN_DAUER_S


def bremsereignisse(df: pd.DataFrame) -> pd.DataFrame:
    """Findet alle Bremsereignisse eines Logs.

    Rueckgabe: DataFrame mit einer Zeile pro Ereignis --
        start_s            Beginn (Sekunden ab Logstart)
        dauer_s            Dauer
        p_max_vorne_bar    Spitzendruck vorne im Ereignis
        p_max_hinten_bar   Spitzendruck hinten im Ereignis
        v_vorher_kmh       GNSS-Geschwindigkeit bei Beginn
        v_nachher_kmh      GNSS-Geschwindigkeit bei Ende
        verzoegerung_m_s2  mittlere Verzoegerung aus der GNSS-Geschwindigkeit
        lat_deg, lon_deg   Position bei Beginn (fuer die Karte im Report)
        fix                1 = Position gueltig, 0 = kein GNSS-Fix
    """
    # Massgeblich ist der jeweils hoehere der beiden Kanaele; NaN-Werte
    # (Kabelbruch-Filter aus daten.py) zaehlen als 0 bar.
    p = df[["p_vorne_bar", "p_hinten_bar"]].max(axis=1).fillna(0.0).to_numpy()
    t = df["t_s"].to_numpy()

    # Zustandsautomat ueber alle Zeilen: ausserhalb eines Ereignisses auf
    # das Ueberschreiten der Startschwelle warten, innerhalb auf das
    # Unterschreiten der Endschwelle.
    grenzen = []
    start_i = None
    for i, wert in enumerate(p):
        if start_i is None:
            if wert > BREMS_START_BAR:
                start_i = i
        elif wert < BREMS_ENDE_BAR:
            grenzen.append((start_i, i))
            start_i = None
    if start_i is not None:  # Ereignis lief beim Dateiende noch
        grenzen.append((start_i, len(p) - 1))

    # Kennzahlen je Ereignis einsammeln; zu kurze Ereignisse verwerfen.
    zeilen = []
    for s, e in grenzen:
        dauer = t[e] - t[s]
        if dauer < BREMS_MIN_DAUER_S:
            continue
        seg = df.iloc[s:e + 1]
        # Fusionierte Geschwindigkeit bevorzugen: Ein Bremsvorgang ist
        # kuerzer als der GNSS-Takt von 1 s, der Rohwert waere zwischen
        # zwei Stuetzstellen nur interpoliert. Rueckhaltetest an LOG_045:
        # 3,86 km/h RMS mit Fusion gegen 4,75 km/h ohne (siehe
        # KALMAN_SIGMA_A_M_S2 in konstanten.py).
        v_spalte = "v_fusion_kmh" if "v_fusion_kmh" in seg.columns else "v_km_h"
        v_vor = float(seg[v_spalte].iloc[0])
        v_nach = float(seg[v_spalte].iloc[-1])
        zeilen.append({
            "start_s": float(t[s]),
            "dauer_s": float(dauer),
            "p_max_vorne_bar": float(seg["p_vorne_bar"].max()),
            "p_max_hinten_bar": float(seg["p_hinten_bar"].max()),
            "v_vorher_kmh": v_vor,
            "v_nachher_kmh": v_nach,
            # Mittlere Verzoegerung aus der GNSS-Geschwindigkeit; ohne Fix
            # steht v auf 0 und der Wert ist nicht aussagekraeftig.
            "verzoegerung_m_s2": (v_vor - v_nach) / 3.6 / dauer,
            "lat_deg": float(seg["lat_deg"].iloc[0]),
            "lon_deg": float(seg["lon_deg"].iloc[0]),
            "fix": int(seg["fix"].iloc[0]),
        })
    return pd.DataFrame(zeilen)
