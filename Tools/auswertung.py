"""
Auswertung der Datenlogger-CSV-Dateien (LOG_nnn.CSV) -- Einstiegspunkt.

Verwendung:
    python auswertung.py LOG_007.CSV      -> wertet diese Datei aus
    python auswertung.py                  -> nimmt die neueste LOG_*.CSV im Ordner
    (oder ueber Messungen/auswertung.bat)

Erzeugt:
    - Zusammenfassung auf der Konsole (Dauer, reale Abtastrate, Fix-Status,
      Schwerkraft-Check, Gyro-Bias, Maximalwerte, GNSS-Qualitaet)
    - Tabelle der erkannten Bremsereignisse
    - <name>_plots.png   : Zeitverlaeufe aller Sensoren
    - <name>_track.png   : GPS-Spur (nur wenn ein Fix vorhanden war)
    - <name>_report.html : interaktiver Report (falls plotly/folium
      installiert sind, siehe report.py)

Die eigentliche Arbeit steckt in den Modulen dieses Ordners:
    konstanten.py   Umrechnungsfaktoren und Schwellwerte (zur Firmware passend)
    daten.py        CSV laden, Rohwerte -> physikalische Einheiten
    gnss.py         1-Hz-Spur extrahieren, Ausreisser/Luecken bereinigen
    bremsen.py      Bremsereignisse erkennen (Hysterese-Detektor)
    konsole.py      Textausgabe (Zusammenfassung, Ereignistabelle)
    diagramme.py    statische PNG-Plots (matplotlib)
    report.py       interaktiver HTML-Report (plotly/folium, optional)

Diese Datei selbst enthaelt nur noch die Ablaufsteuerung.
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt

from daten import lade_csv
from gnss import gnss_spur
from bremsen import bremsereignisse
from konsole import zusammenfassung, drucke_ereignisse
from diagramme import plots, track_plot


def waehle_logdatei() -> Path:
    """Bestimmt die auszuwertende Datei: erstes Kommandozeilenargument oder,
    falls keins angegeben ist, die neueste LOG_*.CSV im aktuellen
    Arbeitsordner (so funktioniert der Aufruf aus dem Messungen-Ordner)."""
    if len(sys.argv) > 1:
        pfad = Path(sys.argv[1])
    else:
        kandidaten = sorted(Path.cwd().glob("LOG_*.CSV"))
        if not kandidaten:
            sys.exit("Keine LOG_*.CSV im aktuellen Ordner -- Datei als Argument angeben.")
        pfad = kandidaten[-1]

    if not pfad.exists():
        sys.exit(f"Datei nicht gefunden: {pfad}")
    return pfad


def main() -> None:
    """Kompletter Auswertelauf fuer eine Logdatei: laden, GNSS-Spur und
    Bremsereignisse berechnen, Konsole/PNGs/HTML-Report ausgeben."""
    pfad = waehle_logdatei()

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
