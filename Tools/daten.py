"""
Einlesen und Aufbereiten einer Logdatei (LOG_nnn.CSV).

Die Firmware schreibt Rohwerte (LSB, ADC-Counts); dieses Modul rechnet sie
in physikalische Einheiten um und haengt die Ergebnisse als zusaetzliche
Spalten an den DataFrame an. Die Rohspalten bleiben unveraendert erhalten,
damit die Umrechnung jederzeit nachvollziehbar bleibt (Thesis-sauber).
"""

from pathlib import Path

import numpy as np
import pandas as pd

from konstanten import (
    IMU_ACCEL_G_PER_LSB, IMU_GYRO_DPS_PER_LSB, ACC400_G_PER_LSB,
    ADC_VREF, ADC_MAX, TEILER, PSS_OFFSET_V, PSS_V_PER_BAR,
    PSS_FEHLER_MIN_V, PSS_FEHLER_MAX_V,
)


def lies_gyro_bias(pfad: Path) -> tuple[float, float, float]:
    """Liest die Bias-Kommentarzeile, die die Firmware beim Start in den
    Dateikopf schreibt ("# gyro_bias;gx;gy;gz", Rohwerte in LSB).

    Die Firmware misst den Gyro-Bias beim Einschalten im Stand (200 Samples
    gemittelt, siehe imu_lsm6dso.c) und legt ihn als Kommentar VOR der
    CSV-Kopfzeile ab -- die Messspalten selbst bleiben Rohwerte, korrigiert
    wird erst hier am PC. Aeltere Dateien ohne diese Zeile bekommen Bias 0
    (keine Korrektur)."""
    with open(pfad, "r", encoding="ascii", errors="replace") as f:
        erste = f.readline()
    if erste.startswith("# gyro_bias"):
        teile = erste.strip().split(";")
        if len(teile) >= 4:
            return float(teile[1]), float(teile[2]), float(teile[3])
    return (0.0, 0.0, 0.0)


def lade_csv(pfad: Path) -> pd.DataFrame:
    """Laedt eine LOG-Datei und ergaenzt physikalische Spalten.

    Erzeugte Spalten:
        t_s                       Zeit in Sekunden ab der ersten Zeile
        imu_ax_g .. imu_az_g      IMU-Beschleunigung [g]
        imu_gx_dps .. imu_gz_dps  Drehrate [Grad/s], Gyro-Bias abgezogen
        imu_a_betrag_g            Betrag der IMU-Beschleunigung [g]
        acc400_x_g .. acc400_z_g  400g-Sensor [g]
        p_vorne_bar, p_hinten_bar Bremsdruck [bar]; unplausible Werte = NaN
        p_vorne_ok, p_hinten_ok   True = Wert im Plausibilitaetsfenster
        lat_deg, lon_deg          GNSS-Position [Grad]
        v_m_s, v_km_h             GNSS-Geschwindigkeit

    Der verwendete Gyro-Bias steht zusaetzlich in df.attrs["gyro_bias_lsb"],
    damit die Konsolen-Zusammenfassung ausweisen kann, ob korrigiert wurde.
    """
    # comment="#" ueberspringt die Bias-Kopfzeile der Firmware.
    df = pd.read_csv(pfad, sep=";", comment="#")

    # Zeitachse in Sekunden ab erster Zeile.
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0

    # IMU in g bzw. Grad/s; Gyro um den beim Start gemessenen Bias korrigiert.
    bias = lies_gyro_bias(pfad)
    for achse in ("ax", "ay", "az"):
        df[f"imu_{achse}_g"] = df[f"imu_{achse}"] * IMU_ACCEL_G_PER_LSB
    for achse, b in zip(("gx", "gy", "gz"), bias):
        df[f"imu_{achse}_dps"] = (df[f"imu_{achse}"] - b) * IMU_GYRO_DPS_PER_LSB
    df.attrs["gyro_bias_lsb"] = bias
    df["imu_a_betrag_g"] = np.sqrt(
        df["imu_ax_g"] ** 2 + df["imu_ay_g"] ** 2 + df["imu_az_g"] ** 2
    )

    # 400g-Sensor in g.
    for achse in ("x", "y", "z"):
        df[f"acc400_{achse}_g"] = df[f"acc400_{achse}"] * ACC400_G_PER_LSB

    # Bremsdruck: Rohwert -> Pin-Spannung -> Sensorspannung -> bar.
    # Unplausible Spannungen (Kabelbruch/Kurzschluss) werden NaN und fallen
    # damit automatisch aus Plots und Statistik (pandas ignoriert NaN).
    for kanal, roh in (("vorne", "p_vorne_raw"), ("hinten", "p_hinten_raw")):
        u_sensor = df[roh] / ADC_MAX * ADC_VREF / TEILER
        gueltig = (u_sensor >= PSS_FEHLER_MIN_V) & (u_sensor <= PSS_FEHLER_MAX_V)
        bar = ((u_sensor - PSS_OFFSET_V) / PSS_V_PER_BAR).clip(lower=0)
        df[f"p_{kanal}_bar"] = bar.where(gueltig)
        df[f"p_{kanal}_ok"] = gueltig

    # GNSS: Festkomma-Rohformate der Firmware in uebliche Einheiten.
    df["lat_deg"] = df["lat_e7"] / 1e7
    df["lon_deg"] = df["lon_e7"] / 1e7
    df["v_m_s"] = df["v_mm_s"] / 1000.0
    df["v_km_h"] = df["v_m_s"] * 3.6
    return df
