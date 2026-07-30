"""
Konsolenausgabe der Auswertung: Zusammenfassung und Ereignistabelle.

Reine Ausgabe-Funktionen ohne eigene Berechnungslogik -- alles, was hier
gedruckt wird, wurde vorher in daten/gnss/bremsen ermittelt. So bleibt
testbar getrennt, WAS berechnet wird und WIE es praesentiert wird.
"""

import pandas as pd

from konstanten import (
    STILLSTAND_SCHWELLE_M_S, BREMS_START_BAR, BREMS_MIN_DAUER_S,
)


def zusammenfassung(df: pd.DataFrame, gnss_stats: dict) -> None:
    """Druckt die Kennzahlen eines Logs: Dauer, reale Abtastrate,
    Sensor-Plausibilitaet (Schwerkraft-Check, Gyro-Bias), GNSS-Qualitaet
    und Bremsdruck-Statistik."""
    dt = df["t_ms"].diff().dropna()
    dauer_s = df["t_s"].iloc[-1]
    print(f"Zeilen:            {len(df)}")
    print(f"Dauer:             {dauer_s:.1f} s")
    # Reale Abtastrate aus den Zeitstempeln -- der direkte Nachweis, ob die
    # Firmware die 50 Hz (20 ms Zeilenabstand) tatsaechlich erreicht hat.
    print(f"Zeilenabstand:     Median {dt.median():.0f} ms "
          f"(min {dt.min():.0f} / max {dt.max():.0f}) "
          f"-> reale Rate ~{1000.0 / dt.median():.1f} Hz")

    # Schwerkraft-Check und Gyro-Bias nur ueber "ruhige" Abschnitte sinnvoll;
    # als einfache Naeherung: alle Zeilen (bei Fahrten spaeter verfeinern).
    print(f"IMU |a|:           Mittel {df['imu_a_betrag_g'].mean():.3f} g "
          f"(Soll im Stand: 1,000 g)")
    bias = [df[f"imu_{a}_dps"].mean() for a in ("gx", "gy", "gz")]
    korr = "nach Bias-Korrektur " if any(df.attrs.get("gyro_bias_lsb", (0, 0, 0))) else ""
    print(f"Gyro-Mittel:       {bias[0]:+.2f} / {bias[1]:+.2f} / {bias[2]:+.2f} dps "
          f"({korr}im Stand ~0 erwartet)")
    acc_max = max(df[f"acc400_{a}_g"].abs().max() for a in ("x", "y", "z"))
    print(f"ADXL373 Spitze:    {acc_max:.1f} g")

    if (df["fix"] > 0).any():
        erster_fix = df.loc[df["fix"] > 0, "t_s"].iloc[0]
        anteil = 100.0 * (df["fix"] > 0).mean()
        print(f"GNSS-Fix:          ab t={erster_fix:.1f} s ({anteil:.0f} % der Zeilen)")
        print(f"Geschwindigkeit:   max {df['v_km_h'].max():.1f} km/h "
              f"(Stillstandsschwelle {STILLSTAND_SCHWELLE_M_S} m/s)")
        # Luecken-Statistik: die Entscheidungsgrundlage dafuer, ob sich
        # spaeter aufwendigere Verfahren (IMU-Fusion) ueberhaupt lohnen.
        luecken = gnss_stats["luecken"]
        laengste = f", laengste {max(luecken):.1f} s" if luecken else ""
        print(f"GNSS-Qualitaet:    {gnss_stats['updates']} Positions-Updates, "
              f"{gnss_stats['ausreisser']} Ausreisser verworfen, "
              f"{len(luecken)} Luecken{laengste}")
    else:
        print("GNSS-Fix:          keiner (Innenraum?)")

    for kanal in ("vorne", "hinten"):
        p = df[f"p_{kanal}_bar"]
        verworfen = int((~df[f"p_{kanal}_ok"]).sum())
        hinweis = (f"{verworfen} Werte unplausibel verworfen (Kabelbruch?)"
                   if verworfen else "ohne Sensor: nur Rauschen offener Pins")
        print(f"Bremsdruck {kanal:<6} Mittel {p.mean():.1f} bar / max {p.max():.1f} bar "
              f"({hinweis})")


def drucke_ereignisse(ereignisse: pd.DataFrame) -> None:
    """Druckt die von bremsen.bremsereignisse() erkannten Ereignisse als
    Tabelle (eine Zeile pro Ereignis) oder einen Hinweis, wenn keine
    gefunden wurden."""
    if ereignisse.empty:
        print(f"Bremsereignisse:   keine (Schwelle {BREMS_START_BAR} bar, "
              f"min. {BREMS_MIN_DAUER_S * 1000:.0f} ms)")
        return
    print(f"Bremsereignisse:   {len(ereignisse)}")
    print("  Nr  Start[s]  Dauer[s]  p_vorne[bar]  p_hinten[bar]  "
          "v vorher->nachher[km/h]  a[m/s^2]")
    for nr, z in enumerate(ereignisse.itertuples(index=False), start=1):
        print(f"  {nr:>2}  {z.start_s:8.1f}  {z.dauer_s:8.2f}  "
              f"{z.p_max_vorne_bar:12.1f}  {z.p_max_hinten_bar:13.1f}  "
              f"{z.v_vorher_kmh:10.1f} -> {z.v_nachher_kmh:5.1f}      "
              f"{z.verzoegerung_m_s2:6.2f}")
