"""
Einfache Auswertung der Datenlogger-CSV-Dateien (LOG_nnn.CSV).

Verwendung:
    python auswertung.py LOG_007.CSV      -> wertet diese Datei aus
    python auswertung.py                  -> nimmt die neueste LOG_*.CSV im Ordner

Erzeugt:
    - Zusammenfassung auf der Konsole (Dauer, reale Abtastrate, Fix-Status,
      Schwerkraft-Check, Gyro-Bias, Maximalwerte, GNSS-Qualitaet)
    - Tabelle der erkannten Bremsereignisse
    - <name>_plots.png   : Zeitverlaeufe aller Sensoren
    - <name>_track.png   : GPS-Spur (nur wenn ein Fix vorhanden war)
    - <name>_report.html : interaktiver Report (falls plotly/folium
      installiert sind, siehe report.py)

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

# Plausibilitaetsfenster wie in der Firmware (brake_pressure.c): Der PSS-140
# liefert gesund immer 0,5..4,725 V. Deutlich darunter = Kabelbruch/keine
# Versorgung, deutlich darueber = Kurzschluss. Solche Werte werden NaN,
# damit ein Kabelbruch nicht als "137 bar Vollbremsung" in den Plots landet.
PSS_FEHLER_MIN_V = 0.35
PSS_FEHLER_MAX_V = 4.75

# GNSS-Bereinigung: Punkte, die eine unmoegliche Geschwindigkeit implizieren
# (Mehrwegempfang, kurzzeitig schlechte Satellitengeometrie), fliegen raus.
# 40 m/s = 144 km/h -- beim Downhill physikalisch nicht erreichbar.
GNSS_MAX_SPRUNG_M_S = 40.0
GNSS_LUECKE_MIN_S = 1.5     # normaler Update-Abstand ist 1 s; ab hier "Luecke"
GNSS_LUECKE_MAX_S = 3.0     # bis hierhin wird linear interpoliert, danach Bruch

# Bremsereignis-Erkennung: Schwellwert mit Hysterese (Beginn ueber 2 bar,
# Ende erst unter 1,5 bar -- verhindert Flattern um eine einzelne Schwelle).
# Die Mindestdauer verwirft einzelne Rauschspitzen.
BREMS_START_BAR = 2.0
BREMS_ENDE_BAR = 1.5
BREMS_MIN_DAUER_S = 0.2


def lies_gyro_bias(pfad: Path) -> tuple[float, float, float]:
    """Liest die Bias-Kommentarzeile, die die Firmware beim Start in den
    Dateikopf schreibt ("# gyro_bias;gx;gy;gz", Rohwerte in LSB).
    Aeltere Dateien ohne diese Zeile bekommen Bias 0 (keine Korrektur)."""
    with open(pfad, "r", encoding="ascii", errors="replace") as f:
        erste = f.readline()
    if erste.startswith("# gyro_bias"):
        teile = erste.strip().split(";")
        if len(teile) >= 4:
            return float(teile[1]), float(teile[2]), float(teile[3])
    return (0.0, 0.0, 0.0)


def lade_csv(pfad: Path) -> pd.DataFrame:
    # comment="#" ueberspringt die Bias-Kopfzeile der Firmware.
    df = pd.read_csv(pfad, sep=";", comment="#")
    # Zeitachse in Sekunden ab erster Zeile
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

    # 400g-Sensor in g
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

    # GNSS
    df["lat_deg"] = df["lat_e7"] / 1e7
    df["lon_deg"] = df["lon_e7"] / 1e7
    df["v_m_s"] = df["v_mm_s"] / 1000.0
    df["v_km_h"] = df["v_m_s"] * 3.6
    return df


def gnss_spur(df: pd.DataFrame):
    """Extrahiert die echten GNSS-Updates (1 Hz) aus den 50-Hz-Zeilen,
    verwirft physikalisch unmoegliche Ausreisser und ueberbrueckt kurze
    Empfangsluecken durch lineare Interpolation.

    Rueckgabe: (spur, statistik)
      spur      DataFrame mit t_s, lat_deg, lon_deg, ost_m, nord_m, v_km_h,
                interp (True = interpolierter Ersatzpunkt) -- oder None,
                wenn keine auswertbare Spur existiert
      statistik dict: updates, ausreisser, luecken (Liste der Laengen in s)
    """
    statistik = {"updates": 0, "ausreisser": 0, "luecken": []}
    fix = df[df["fix"] > 0]
    if fix.empty or fix["lat_e7"].nunique() < 2:
        return None, statistik

    # Der Logger schreibt 50 Zeilen/s, das GNSS liefert aber nur 1 Position/s:
    # aufeinanderfolgende Duplikate verwerfen, nur echte Updates behalten.
    neu = (fix["lat_e7"].diff() != 0) | (fix["lon_e7"].diff() != 0)
    neu.iloc[0] = True
    fix = fix[neu]
    if len(fix) < 2:
        return None, statistik
    statistik["updates"] = len(fix)

    # Grad -> Meter relativ zum Startpunkt (lokale Ebene, fuer kleine
    # Gebiete voellig ausreichend genau).
    lat0 = np.radians(fix["lat_deg"].iloc[0])
    ost = ((fix["lon_deg"] - fix["lon_deg"].iloc[0]) * 111320.0 * np.cos(lat0)).to_numpy()
    nord = ((fix["lat_deg"] - fix["lat_deg"].iloc[0]) * 110540.0).to_numpy()
    t = fix["t_s"].to_numpy()
    v = fix["v_km_h"].to_numpy()
    lat = fix["lat_deg"].to_numpy()
    lon = fix["lon_deg"].to_numpy()

    # Ausreisser-Rejection: Jeder Punkt muss vom zuletzt akzeptierten aus
    # mit plausibler Geschwindigkeit erreichbar sein ("Teleport"-Spruenge
    # hin und zurueck werden so beide verworfen).
    behalten = [0]
    for i in range(1, len(t)):
        j = behalten[-1]
        dt = max(t[i] - t[j], 1e-3)
        d = float(np.hypot(ost[i] - ost[j], nord[i] - nord[j]))
        if d / dt <= GNSS_MAX_SPRUNG_M_S:
            behalten.append(i)
        else:
            statistik["ausreisser"] += 1
    if len(behalten) < 2:
        return None, statistik
    idx = np.array(behalten)
    t, v, ost, nord = t[idx], v[idx], ost[idx], nord[idx]
    lat, lon = lat[idx], lon[idx]

    # Luecken behandeln: kurze linear auffuellen (als interpoliert markiert),
    # lange bleiben eine sichtbare Unterbrechung in der Spur.
    punkte = []
    for i in range(len(t)):
        punkte.append((t[i], lat[i], lon[i], ost[i], nord[i], v[i], False))
        if i + 1 < len(t):
            dt = t[i + 1] - t[i]
            if dt > GNSS_LUECKE_MIN_S:
                statistik["luecken"].append(float(dt))
                if dt <= GNSS_LUECKE_MAX_S:
                    n = max(int(round(dt)), 2)  # 1-s-Raster nachbilden
                    for k in range(1, n):
                        f = k / n
                        punkte.append((t[i] + f * dt,
                                       lat[i] + f * (lat[i + 1] - lat[i]),
                                       lon[i] + f * (lon[i + 1] - lon[i]),
                                       ost[i] + f * (ost[i + 1] - ost[i]),
                                       nord[i] + f * (nord[i + 1] - nord[i]),
                                       v[i] + f * (v[i + 1] - v[i]),
                                       True))
    spur = pd.DataFrame(punkte, columns=["t_s", "lat_deg", "lon_deg",
                                         "ost_m", "nord_m", "v_km_h", "interp"])
    return spur, statistik


def bremsereignisse(df: pd.DataFrame) -> pd.DataFrame:
    """Erkennt Bremsereignisse: Beginn, wenn vorne ODER hinten ueber
    BREMS_START_BAR, Ende erst unter BREMS_ENDE_BAR (Hysterese), Ereignisse
    kuerzer als BREMS_MIN_DAUER_S gelten als Rauschspitze und entfallen."""
    p = df[["p_vorne_bar", "p_hinten_bar"]].max(axis=1).fillna(0.0).to_numpy()
    t = df["t_s"].to_numpy()

    grenzen = []
    start_i = None
    for i, wert in enumerate(p):
        if start_i is None:
            if wert > BREMS_START_BAR:
                start_i = i
        elif wert < BREMS_ENDE_BAR:
            grenzen.append((start_i, i))
            start_i = None
    if start_i is not None:
        grenzen.append((start_i, len(p) - 1))

    zeilen = []
    for s, e in grenzen:
        dauer = t[e] - t[s]
        if dauer < BREMS_MIN_DAUER_S:
            continue
        seg = df.iloc[s:e + 1]
        v_vor = float(seg["v_km_h"].iloc[0])
        v_nach = float(seg["v_km_h"].iloc[-1])
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


def drucke_ereignisse(ereignisse: pd.DataFrame) -> None:
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


def zusammenfassung(df: pd.DataFrame, gnss_stats: dict) -> None:
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
        # Lücken-Statistik: die Entscheidungsgrundlage dafuer, ob sich
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


def track_plot(spur: pd.DataFrame, basis: Path) -> None:
    if spur is None:
        return  # kein Fix oder keine Bewegung -> keine sinnvolle Spur

    # Segmente zwischen aufeinanderfolgenden Punkten aufteilen: echte
    # Segmente farbig nach Geschwindigkeit, interpolierte grau gestrichelt;
    # ueber lange Luecken wird gar nicht verbunden (sichtbarer Bruch).
    p = spur[["ost_m", "nord_m"]].to_numpy()
    t = spur["t_s"].to_numpy()
    v = spur["v_km_h"].to_numpy()
    interp = spur["interp"].to_numpy()

    seg_echt, seg_interp, v_echt = [], [], []
    for i in range(len(p) - 1):
        if t[i + 1] - t[i] > GNSS_LUECKE_MAX_S:
            continue
        seg = [p[i], p[i + 1]]
        if interp[i] or interp[i + 1]:
            seg_interp.append(seg)
        else:
            seg_echt.append(seg)
            v_echt.append(0.5 * (v[i] + v[i + 1]))

    fig, a = plt.subplots(figsize=(8, 8))
    if seg_echt:
        linie = LineCollection(seg_echt, cmap="viridis", linewidths=2)
        linie.set_array(np.array(v_echt))
        a.add_collection(linie)
        fig.colorbar(linie, ax=a, label="v [km/h]")
    if seg_interp:
        a.add_collection(LineCollection(seg_interp, colors="gray",
                                        linestyles="--", linewidths=1.5))

    a.plot(spur["ost_m"].iloc[0], spur["nord_m"].iloc[0], "o", color="green",
           markersize=10, label="Start")
    a.plot(spur["ost_m"].iloc[-1], spur["nord_m"].iloc[-1], "s", color="red",
           markersize=8, label="Ende")

    n_interp = int(spur["interp"].sum())
    zusatz = f", {n_interp} interpoliert" if n_interp else ""
    a.set_xlabel("Ost [m]")
    a.set_ylabel("Nord [m]")
    a.set_title(f"GPS-Spur {basis.name} "
                f"({int((~spur['interp']).sum())} GNSS-Positionen{zusatz})")
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
    spur, gnss_stats = gnss_spur(df)
    zusammenfassung(df, gnss_stats)
    ereignisse = bremsereignisse(df)
    drucke_ereignisse(ereignisse)
    plots(df, pfad)
    track_plot(spur, pfad)

    # Interaktiver HTML-Report (plotly/folium); ohne die Pakete laeuft die
    # restliche Auswertung trotzdem durch.
    try:
        import report
        report.erzeuge_report(df, spur, ereignisse, pfad)
    except ImportError:
        print("Hinweis: plotly/folium fehlen -- HTML-Report uebersprungen "
              "(python -m pip install plotly folium).")

    plt.show()


if __name__ == "__main__":
    main()
