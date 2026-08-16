"""
Bestimmt FAHRT_ACHSE und FAHRT_VORZEICHEN aus echten Fahrmessungen.

Verwendung:
    python kalibrierung.py LOG_036.CSV LOG_037.CSV
    python kalibrierung.py                 -> alle LOG_*.CSV im Ordner

HINTERGRUND
Der Madgwick-Lagefilter liefert die neigungsbereinigte Beschleunigung in
allen drei Sensorachsen (a_lin_x/y/z_ms2). Welche davon in Fahrtrichtung
zeigt und mit welchem Vorzeichen, haengt allein davon ab, wie das Board am
Rad montiert ist -- das ist aus den Daten nicht ableitbar, solange man
keine unabhaengige Referenz hat.

Die Referenz ist die GNSS-Geschwindigkeit: Ihre zeitliche Ableitung IST
die Laengsbeschleunigung. Beim Anfahren positiv, beim Bremsen negativ.
Man vergleicht also jede der sechs Moeglichkeiten (drei Achsen mal zwei
Vorzeichen) mit dieser Referenz und nimmt die mit der besten
Uebereinstimmung.

METHODE
1. GNSS-Referenz: zentrale Differenz der echten (nicht interpolierten)
   Geschwindigkeitsupdates -> a_gnss[k] auf dem 1-Hz-Raster.
2. IMU-Vergleichswert: die 50-Hz-Beschleunigung ueber dasselbe Intervall
   mitteln. Das ist wichtig -- ein Momentanwert bei t_k gegen eine ueber
   eine Sekunde gebildete Differenz zu stellen waere Aepfel mit Birnen.
3. Fuer jede Achse: Korrelationskoeffizient und Steigung einer Regression
   a_gnss = m * a_imu. Das Vorzeichen ergibt sich aus dem Vorzeichen der
   Korrelation, die Achse aus deren Betrag.

Die Steigung m ist zusaetzlich aufschlussreich: Bei sauberer Kalibrierung
liegt sie nahe 1. Deutlich kleiner heisst, dass die Fahrtrichtung schraeg
zwischen zwei Achsen liegt (Montagewinkel) oder dass der Lagefilter die
Schwerkraft nicht vollstaendig herausrechnet.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

from daten import lade_csv
from gnss import gnss_spur
from fusion import lagefilter
from konstanten import STILLSTAND_SCHWELLE_M_S

ACHSEN = ("x", "y", "z")

# Nur Intervalle verwenden, in denen sich ueberhaupt etwas tut. Im Stand
# rauscht die GNSS-Geschwindigkeit um bis zu STILLSTAND_SCHWELLE_M_S, und
# dieses Rauschen wuerde die Korrelation nur verwaessern.
MIN_ABS_A = 0.3          # m/s^2 -- darunter gilt das Intervall als ereignislos
MIN_INTERVALLE = 10      # weniger Stuetzstellen sind statistisch wertlos


def referenz_und_imu(pfad: Path):
    """Liefert (a_gnss, a_imu_je_achse, name) fuer eine Logdatei.

    a_gnss:  GNSS-Laengsbeschleunigung je Intervall [m/s^2]
    a_imu:   dict Achse -> ueber dasselbe Intervall gemittelte IMU-Werte
    """
    df = lade_csv(pfad)
    spur, _ = gnss_spur(df)
    if spur is None or spur.empty:
        return None, None, pfad.name

    df = lagefilter(df)

    echt = spur[~spur["interp"]]
    t_g = echt["t_s"].to_numpy()
    v_g = echt["v_km_h"].to_numpy() / 3.6
    if len(t_g) < MIN_INTERVALLE + 1:
        return None, None, pfad.name

    # Zentrale Differenz auf dem GNSS-Raster: a[k] gilt fuer das Intervall
    # zwischen t[k-1] und t[k+1].
    a_gnss = []
    fenster = []
    for k in range(1, len(t_g) - 1):
        dt = t_g[k + 1] - t_g[k - 1]
        if dt <= 0:
            continue
        a_gnss.append((v_g[k + 1] - v_g[k - 1]) / dt)
        fenster.append((t_g[k - 1], t_g[k + 1]))

    a_gnss = np.asarray(a_gnss)

    # IMU ueber genau dieselben Fenster mitteln.
    t_imu = df["t_s"].to_numpy()
    a_imu = {}
    for achse in ACHSEN:
        werte = df[f"a_lin_{achse}_ms2"].to_numpy()
        gemittelt = np.empty(len(fenster))
        for i, (t0, t1) in enumerate(fenster):
            maske = (t_imu >= t0) & (t_imu <= t1)
            gemittelt[i] = werte[maske].mean() if maske.any() else np.nan
        a_imu[achse] = gemittelt

    # Ereignislose Intervalle verwerfen (Stand, Konstantfahrt).
    gueltig = np.abs(a_gnss) >= MIN_ABS_A
    for achse in ACHSEN:
        gueltig &= np.isfinite(a_imu[achse])

    if gueltig.sum() < MIN_INTERVALLE:
        return None, None, pfad.name

    return a_gnss[gueltig], {a: a_imu[a][gueltig] for a in ACHSEN}, pfad.name


def bewerte(a_gnss, a_imu) -> pd.DataFrame:
    """Korrelation und Regressionssteigung je Achse."""
    zeilen = []
    for achse in ACHSEN:
        x = a_imu[achse]
        y = a_gnss
        if np.std(x) < 1e-9:
            r, m = 0.0, 0.0
        else:
            r = float(np.corrcoef(x, y)[0, 1])
            # Steigung durch den Ursprung: beide Groessen sind echte
            # Beschleunigungen, ein Achsenabschnitt waere unphysikalisch.
            m = float(np.dot(x, y) / np.dot(x, x))
        zeilen.append({"achse": achse, "r": r, "steigung": m,
                       "vorzeichen": +1.0 if r >= 0 else -1.0})
    return pd.DataFrame(zeilen)


def main() -> None:
    if len(sys.argv) > 1:
        pfade = [Path(p) for p in sys.argv[1:]]
    else:
        pfade = sorted(Path.cwd().glob("LOG_*.CSV"))
    if not pfade:
        sys.exit("Keine LOG_*.CSV gefunden -- Dateien als Argument angeben.")

    alle_g, alle_i = [], {a: [] for a in ACHSEN}
    verwendet = []

    for pfad in pfade:
        if not pfad.exists():
            print(f"  {pfad.name}: nicht gefunden -- uebersprungen")
            continue
        a_g, a_i, name = referenz_und_imu(pfad)
        if a_g is None:
            print(f"  {name}: zu wenige brauchbare Intervalle -- uebersprungen")
            continue

        tab = bewerte(a_g, a_i)
        best = tab.loc[tab["r"].abs().idxmax()]
        print(f"\n=== {name} ({len(a_g)} Intervalle) ===")
        for _, z in tab.iterrows():
            marke = "  <--" if z["achse"] == best["achse"] else ""
            print(f"  Achse {z['achse']}:  r = {z['r']:+.3f}   "
                  f"Steigung = {z['steigung']:+.2f}{marke}")

        alle_g.append(a_g)
        for a in ACHSEN:
            alle_i[a].append(a_i[a])
        verwendet.append(name)

    if not verwendet:
        sys.exit("\nKeine Datei lieferte genug Fahrdynamik fuer eine Kalibrierung.")

    # Gemeinsame Auswertung ueber alle Fahrten -- mehr Stuetzstellen,
    # weniger Zufall.
    g = np.concatenate(alle_g)
    i = {a: np.concatenate(alle_i[a]) for a in ACHSEN}
    tab = bewerte(g, i)
    best = tab.loc[tab["r"].abs().idxmax()]

    print("\n" + "=" * 62)
    print(f"GESAMT ueber {len(verwendet)} Datei(en), {len(g)} Intervalle")
    print("=" * 62)
    for _, z in tab.iterrows():
        print(f"  Achse {z['achse']}:  r = {z['r']:+.3f}   "
              f"Steigung = {z['steigung']:+.2f}")

    print(f"\n  -> FAHRT_ACHSE = \"{best['achse']}\"")
    print(f"  -> FAHRT_VORZEICHEN = {best['vorzeichen']:+.1f}")
    print(f"     (Korrelation {best['r']:+.3f}, Steigung {best['steigung']:+.2f})")

    if abs(best["r"]) < 0.5:
        print("\n  ACHTUNG: schwache Korrelation. Moegliche Ursachen: zu wenig\n"
              "  Fahrdynamik in den Daten, schraege Montage (Fahrtrichtung\n"
              "  zwischen zwei Achsen) oder ein Lagefilter, der die\n"
              "  Schwerkraft nicht sauber herausrechnet.")
    elif abs(best["steigung"]) < 0.5 or abs(best["steigung"]) > 2.0:
        print("\n  HINWEIS: Die Steigung liegt weit von 1 entfernt. Richtung und\n"
              "  Vorzeichen stimmen, der Betrag der IMU-Beschleunigung passt\n"
              "  aber nicht zur GNSS-Referenz -- typisch fuer eine schraege\n"
              "  Montage. Das Prozessrauschen des Kalman sollte dann nicht zu\n"
              "  weit gesenkt werden.")


if __name__ == "__main__":
    main()
