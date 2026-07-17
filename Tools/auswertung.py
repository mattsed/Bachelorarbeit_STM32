"""
Einfache Auswertung der Datenlogger-CSV-Dateien (LOG_nnn.CSV).

Verwendung:
    python auswertung.py LOG_007.CSV      -> wertet diese Datei aus
    python auswertung.py                  -> nimmt die neueste LOG_*.CSV im Ordner

Erzeugt:
    - Zusammenfassung auf der Konsole (Dauer, reale Abtastrate, Fix-Status,
      Schwerkraft-Check, Gyro-Bias, Maximalwerte)
    - <name>_plots.png   : Zeitverlaeufe aller Sensoren
    - <name>_track.png   : GPS-Spur (nur wenn ein Fix vorhanden war)

Umrechnungsfaktoren passend zur Firmware-Konfiguration:
    LSM6DSO:  +/-16 g  -> 0,488 mg/LSB   | +/-2000 dps -> 70 mdps/LSB
    ADXL373:  200 mg/LSB
    Bremsdruck: 12-Bit-ADC, 3,3 V Referenz, Teiler 33/48, PSS-140
                (Offset 0,5 V, 28,571 mV/bar)
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ---------------------------------------------------------------- Konstanten
IMU_ACCEL_G_PER_LSB = 0.488e-3      # +/-16 g
IMU_GYRO_DPS_PER_LSB = 70e-3        # +/-2000 dps
ACC400_G_PER_LSB = 0.2              # 200 mg/LSB
ADC_VREF = 3.3
ADC_MAX = 4095.0
TEILER = 33.0 / 48.0                # Spannungsteiler 15k/33k
PSS_OFFSET_V = 0.5
PSS_V_PER_BAR = 0.028571
STILLSTAND_SCHWELLE_M_S = 0.5       # GNSS-Rauschen im Stand (siehe Protokoll)


def lade_csv(pfad: Path) -> pd.DataFrame:
    df = pd.read_csv(pfad, sep=";")
    # Zeitachse in Sekunden ab erster Zeile
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0

    # IMU in g bzw. Grad/s
    for achse in ("ax", "ay", "az"):
        df[f"imu_{achse}_g"] = df[f"imu_{achse}"] * IMU_ACCEL_G_PER_LSB
    for achse in ("gx", "gy", "gz"):
        df[f"imu_{achse}_dps"] = df[f"imu_{achse}"] * IMU_GYRO_DPS_PER_LSB
    df["imu_a_betrag_g"] = np.sqrt(
        df["imu_ax_g"] ** 2 + df["imu_ay_g"] ** 2 + df["imu_az_g"] ** 2
    )

    # 400g-Sensor in g
    for achse in ("x", "y", "z"):
        df[f"acc400_{achse}_g"] = df[f"acc400_{achse}"] * ACC400_G_PER_LSB

    # Bremsdruck: Rohwert -> Pin-Spannung -> Sensorspannung -> bar
    for kanal, roh in (("vorne", "p_vorne_raw"), ("hinten", "p_hinten_raw")):
        u_sensor = df[roh] / ADC_MAX * ADC_VREF / TEILER
        df[f"p_{kanal}_bar"] = ((u_sensor - PSS_OFFSET_V) / PSS_V_PER_BAR).clip(lower=0)

    # GNSS
    df["lat_deg"] = df["lat_e7"] / 1e7
    df["lon_deg"] = df["lon_e7"] / 1e7
    df["v_m_s"] = df["v_mm_s"] / 1000.0
    df["v_km_h"] = df["v_m_s"] * 3.6
    return df


def zusammenfassung(df: pd.DataFrame) -> None:
    dt = df["t_ms"].diff().dropna()
    dauer_s = df["t_s"].iloc[-1]
    print(f"Zeilen:            {len(df)}")
    print(f"Dauer:             {dauer_s:.1f} s")
    print(f"Zeilenabstand:     Median {dt.median():.0f} ms "
          f"(min {dt.min():.0f} / max {dt.max():.0f}) "
          f"-> reale Rate ~{1000.0 / dt.median():.1f} Hz")

    # Schwerkraft-Check und Gyro-Bias nur ueber "ruhige" Abschnitte sinnvoll;
    # als einfache Naeherung: alle Zeilen (bei Fahrten spaeter verfeinern).
    print(f"IMU |a|:           Mittel {df['imu_a_betrag_g'].mean():.3f} g "
          f"(Soll im Stand: 1,000 g)")
    bias = [df[f"imu_{a}_dps"].mean() for a in ("gx", "gy", "gz")]
    print(f"Gyro-Mittel:       {bias[0]:+.2f} / {bias[1]:+.2f} / {bias[2]:+.2f} dps "
          f"(Bias, im Stand ~0 erwartet)")
    acc_max = max(df[f"acc400_{a}_g"].abs().max() for a in ("x", "y", "z"))
    print(f"ADXL373 Spitze:    {acc_max:.1f} g")

    if (df["fix"] > 0).any():
        erster_fix = df.loc[df["fix"] > 0, "t_s"].iloc[0]
        anteil = 100.0 * (df["fix"] > 0).mean()
        print(f"GNSS-Fix:          ab t={erster_fix:.1f} s ({anteil:.0f} % der Zeilen)")
        print(f"Geschwindigkeit:   max {df['v_km_h'].max():.1f} km/h "
              f"(Stillstandsschwelle {STILLSTAND_SCHWELLE_M_S} m/s)")
    else:
        print("GNSS-Fix:          keiner (Innenraum?)")

    for kanal in ("vorne", "hinten"):
        p = df[f"p_{kanal}_bar"]
        print(f"Bremsdruck {kanal:<6} Mittel {p.mean():.1f} bar / max {p.max():.1f} bar "
              f"(ohne Sensor: nur Rauschen offener Pins)")


def plots(df: pd.DataFrame, basis: Path) -> None:
    fig, achsen = plt.subplots(5, 1, figsize=(12, 14), sharex=True)

    a = achsen[0]
    for achse in ("ax", "ay", "az"):
        a.plot(df["t_s"], df[f"imu_{achse}_g"], label=achse)
    a.plot(df["t_s"], df["imu_a_betrag_g"], label="|a|", color="black", lw=0.8)
    a.set_ylabel("IMU [g]")
    a.legend(loc="upper right", ncols=4)
    a.grid(True, alpha=0.3)

    a = achsen[1]
    for achse in ("gx", "gy", "gz"):
        a.plot(df["t_s"], df[f"imu_{achse}_dps"], label=achse)
    a.set_ylabel("Drehrate [°/s]")
    a.legend(loc="upper right", ncols=3)
    a.grid(True, alpha=0.3)

    a = achsen[2]
    for achse in ("x", "y", "z"):
        a.plot(df["t_s"], df[f"acc400_{achse}_g"], label=achse)
    a.set_ylabel("ADXL373 [g]")
    a.legend(loc="upper right", ncols=3)
    a.grid(True, alpha=0.3)

    a = achsen[3]
    a.plot(df["t_s"], df["p_vorne_bar"], label="vorne")
    a.plot(df["t_s"], df["p_hinten_bar"], label="hinten")
    a.set_ylabel("Bremsdruck [bar]")
    a.legend(loc="upper right", ncols=2)
    a.grid(True, alpha=0.3)

    a = achsen[4]
    a.plot(df["t_s"], df["v_km_h"], color="tab:green")
    a.set_ylabel("GNSS v [km/h]")
    a.set_xlabel("Zeit [s]")
    a.grid(True, alpha=0.3)

    fig.suptitle(basis.name)
    fig.tight_layout()
    ziel = basis.with_name(basis.stem + "_plots.png")
    fig.savefig(ziel, dpi=150)
    print(f"Plot gespeichert:  {ziel}")


def track_plot(df: pd.DataFrame, basis: Path) -> None:
    fix = df[df["fix"] > 0].copy()
    if fix.empty or fix["lat_deg"].nunique() < 2:
        return  # kein Fix oder keine Bewegung -> keine sinnvolle Spur

    # Der Logger schreibt 50 Zeilen/s, das GNSS liefert aber nur 1 Position/s:
    # aufeinanderfolgende Duplikate verwerfen, damit die Linie der zeitlichen
    # Abfolge der echten GNSS-Updates folgt.
    neu = (fix["lat_e7"].diff() != 0) | (fix["lon_e7"].diff() != 0)
    neu.iloc[0] = True
    fix = fix[neu]
    if len(fix) < 2:
        return

    # Grad -> Meter relativ zum Startpunkt (lokale Ebene, fuer kleine
    # Gebiete voellig ausreichend genau).
    lat0 = np.radians(fix["lat_deg"].iloc[0])
    fix["ost_m"] = (fix["lon_deg"] - fix["lon_deg"].iloc[0]) * 111320.0 * np.cos(lat0)
    fix["nord_m"] = (fix["lat_deg"] - fix["lat_deg"].iloc[0]) * 110540.0

    # Linie in Zeitreihenfolge, Segmente nach Geschwindigkeit eingefaerbt.
    punkte = np.column_stack([fix["ost_m"], fix["nord_m"]])
    segmente = np.stack([punkte[:-1], punkte[1:]], axis=1)
    v_segment = 0.5 * (fix["v_km_h"].to_numpy()[:-1] + fix["v_km_h"].to_numpy()[1:])

    fig, a = plt.subplots(figsize=(8, 8))
    linie = LineCollection(segmente, cmap="viridis", linewidths=2)
    linie.set_array(v_segment)
    a.add_collection(linie)
    fig.colorbar(linie, ax=a, label="v [km/h]")

    a.plot(fix["ost_m"].iloc[0], fix["nord_m"].iloc[0], "o", color="green",
           markersize=10, label="Start")
    a.plot(fix["ost_m"].iloc[-1], fix["nord_m"].iloc[-1], "s", color="red",
           markersize=8, label="Ende")

    a.set_xlabel("Ost [m]")
    a.set_ylabel("Nord [m]")
    a.set_title(f"GPS-Spur {basis.name} ({len(fix)} GNSS-Positionen)")
    a.set_aspect("equal", adjustable="datalim")
    a.autoscale()
    a.legend(loc="best")
    a.grid(True, alpha=0.3)
    fig.tight_layout()
    ziel = basis.with_name(basis.stem + "_track.png")
    fig.savefig(ziel, dpi=150)
    print(f"Spur gespeichert:  {ziel}")


def main() -> None:
    if len(sys.argv) > 1:
        pfad = Path(sys.argv[1])
    else:
        # Ohne Argument: neueste LOG-Datei im aktuellen Arbeitsordner
        # (z. B. im Messungen-Ordner starten).
        kandidaten = sorted(Path.cwd().glob("LOG_*.CSV"))
        if not kandidaten:
            sys.exit("Keine LOG_*.CSV im aktuellen Ordner -- Datei als Argument angeben.")
        pfad = kandidaten[-1]

    if not pfad.exists():
        sys.exit(f"Datei nicht gefunden: {pfad}")

    print(f"=== Auswertung: {pfad.name} ===")
    df = lade_csv(pfad)
    zusammenfassung(df)
    plots(df, pfad)
    track_plot(df, pfad)
    plt.show()


if __name__ == "__main__":
    main()
