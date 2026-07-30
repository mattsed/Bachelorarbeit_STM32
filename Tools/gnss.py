"""
GNSS-Spuraufbereitung: aus den 50-Hz-Logzeilen eine saubere 1-Hz-Spur machen.

Der Logger schreibt 50 Zeilen pro Sekunde, das GNSS-Modul liefert aber nur
eine Position pro Sekunde -- jede Position steht also ~50-mal wiederholt in
der Datei. Dieses Modul extrahiert die echten Updates und bereinigt sie in
drei Schritten:

  1. Dedupe:      aufeinanderfolgende identische Positionen verwerfen.
  2. Ausreisser:  Punkte, die vom zuletzt akzeptierten Punkt aus nur mit
                  physikalisch unmoeglicher Geschwindigkeit erreichbar
                  waeren (Mehrwegempfang, "Teleport"-Spruenge), fliegen raus.
  3. Luecken:     kurze Empfangsluecken (<= GNSS_LUECKE_MAX_S) werden linear
                  aufgefuellt und als interpoliert markiert; laengere bleiben
                  eine sichtbare Unterbrechung in der Spur.

Die Lueckenstatistik ist zugleich die dokumentierte Entscheidungsgrundlage
dafuer, ob aufwendigere Verfahren (IMU-Fusion/Kalman) noetig waeren.
"""

import numpy as np
import pandas as pd

from konstanten import GNSS_MAX_SPRUNG_M_S, GNSS_LUECKE_MIN_S, GNSS_LUECKE_MAX_S


def gnss_spur(df: pd.DataFrame):
    """Extrahiert die bereinigte GNSS-Spur aus einem geladenen Log.

    Rueckgabe: (spur, statistik)
      spur      DataFrame mit t_s, lat_deg, lon_deg, ost_m, nord_m, v_km_h,
                interp (True = interpolierter Ersatzpunkt) -- oder None,
                wenn keine auswertbare Spur existiert (kein Fix / keine
                Bewegung).
      statistik dict: updates (Anzahl echter Positions-Updates),
                ausreisser (Anzahl verworfener Punkte),
                luecken (Liste der Lueckenlaengen in s)
    """
    statistik = {"updates": 0, "ausreisser": 0, "luecken": []}
    fix = df[df["fix"] > 0]
    if fix.empty or fix["lat_e7"].nunique() < 2:
        return None, statistik

    # Schritt 1 -- Dedupe: nur Zeilen behalten, in denen sich die Position
    # gegenueber der Vorzeile tatsaechlich geaendert hat.
    neu = (fix["lat_e7"].diff() != 0) | (fix["lon_e7"].diff() != 0)
    neu.iloc[0] = True
    fix = fix[neu]
    if len(fix) < 2:
        return None, statistik
    statistik["updates"] = len(fix)

    # Grad -> Meter relativ zum Startpunkt (lokale Ebene; fuer die kleinen
    # Gebiete einer Abfahrt voellig ausreichend genau).
    lat0 = np.radians(fix["lat_deg"].iloc[0])
    ost = ((fix["lon_deg"] - fix["lon_deg"].iloc[0]) * 111320.0 * np.cos(lat0)).to_numpy()
    nord = ((fix["lat_deg"] - fix["lat_deg"].iloc[0]) * 110540.0).to_numpy()
    t = fix["t_s"].to_numpy()
    v = fix["v_km_h"].to_numpy()
    lat = fix["lat_deg"].to_numpy()
    lon = fix["lon_deg"].to_numpy()

    # Schritt 2 -- Ausreisser-Rejection: Jeder Punkt muss vom zuletzt
    # AKZEPTIERTEN Punkt aus mit plausibler Geschwindigkeit erreichbar sein.
    # Dadurch werden "Teleport"-Spruenge hin und zurueck beide verworfen
    # (ein einfacher Vergleich mit dem direkten Vorgaenger wuerde nur den
    # Hinsprung erkennen).
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

    # Schritt 3 -- Luecken behandeln: kurze linear auffuellen (im 1-s-Raster,
    # als interpoliert markiert), lange bleiben eine sichtbare Unterbrechung.
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
