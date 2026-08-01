"""
Interaktiver HTML-Report fuer die Datenlogger-Auswertung.

Wird von auswertung.py aufgerufen und erzeugt <name>_report.html mit:
    - zoombaren Zeitreihen (plotly): Bremsdruck, Beschleunigung, Geschwindigkeit
    - einer Karte (folium/OpenStreetMap) mit der Spur, eingefaerbt nach dem
      maximalen Bremsdruck des jeweiligen Streckenabschnitts, plus Markern
      fuer die erkannten Bremsereignisse
    - der Ereignistabelle

Der plotly-Teil ist komplett in die Datei eingebettet (funktioniert offline);
nur die Kartenkacheln der folium-Karte brauchen eine Internetverbindung.
"""

import html
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import folium
import branca.colormap as cm

# Gleiche Schwellen wie in der uebrigen Auswertung (in konstanten.py
# dokumentiert).
from konstanten import GNSS_LUECKE_MAX_S


def _zeitreihen(df: pd.DataFrame) -> str:
    """Drei gestapelte, gemeinsam zoombare Zeitreihen als HTML-Fragment."""
    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.06,
        subplot_titles=("Bremsdruck [bar]", "Beschleunigung [g]",
                        "GNSS-Geschwindigkeit [km/h]"))

    fig.add_trace(go.Scattergl(x=df["t_s"], y=df["p_vorne_bar"],
                               name="Bremse vorne", line=dict(color="#d62728")),
                  row=1, col=1)
    fig.add_trace(go.Scattergl(x=df["t_s"], y=df["p_hinten_bar"],
                               name="Bremse hinten", line=dict(color="#1f77b4")),
                  row=1, col=1)

    fig.add_trace(go.Scattergl(x=df["t_s"], y=df["imu_a_betrag_g"],
                               name="IMU |a|", line=dict(color="#2ca02c")),
                  row=2, col=1)
    acc400_betrag = np.sqrt(df["acc400_x_g"] ** 2 + df["acc400_y_g"] ** 2 +
                            df["acc400_z_g"] ** 2)
    fig.add_trace(go.Scattergl(x=df["t_s"], y=acc400_betrag,
                               name="ADXL373 |a|", line=dict(color="#9467bd")),
                  row=2, col=1)

    fig.add_trace(go.Scattergl(x=df["t_s"], y=df["v_km_h"],
                               name="v GNSS", line=dict(color="#ff7f0e")),
                  row=3, col=1)
    # Fusionierte Geschwindigkeit (falls fusion.py gelaufen ist).
    if "v_fusion_kmh" in df.columns:
        fig.add_trace(go.Scattergl(x=df["t_s"], y=df["v_fusion_kmh"],
                                   name="v Fusion (IMU+GNSS)",
                                   line=dict(color="#8c564b")),
                      row=3, col=1)

    fig.update_layout(height=720, hovermode="x unified",
                      legend=dict(orientation="h", y=1.06),
                      margin=dict(l=60, r=20, t=60, b=40))
    fig.update_xaxes(title_text="Zeit [s]", row=3, col=1)
    # include_plotlyjs=True bettet die Bibliothek ein -> Report ist offline nutzbar.
    return fig.to_html(full_html=False, include_plotlyjs=True)


def _karte(df: pd.DataFrame, spur: pd.DataFrame,
           ereignisse: pd.DataFrame) -> str | None:
    """Spur auf OpenStreetMap, Segmente nach max. Bremsdruck eingefaerbt."""
    if spur is None:
        return None

    lat = spur["lat_deg"].to_numpy()
    lon = spur["lon_deg"].to_numpy()
    t = spur["t_s"].to_numpy()
    interp = spur["interp"].to_numpy()

    # Maximaler Bremsdruck (vorne oder hinten) je Spurabschnitt: Fenster vom
    # jeweiligen Punkt bis zum naechsten (die 50-Hz-Zeilen dazwischen).
    p_max = df[["p_vorne_bar", "p_hinten_bar"]].max(axis=1).fillna(0.0)
    t_df = df["t_s"].to_numpy()
    werte = []
    for i in range(len(t) - 1):
        maske = (t_df >= t[i]) & (t_df < t[i + 1])
        werte.append(float(p_max[maske].max()) if maske.any() else 0.0)

    figur = folium.Figure(height=520)
    # Kachelquelle: Carto statt OpenStreetMap-Standard. Die ehrenamtlich
    # betriebenen OSM-Server blockieren Anfragen ohne Referer -- und genau
    # so fragt ein lokal geoeffnetes file://-HTML an (403 "Access blocked").
    # Die Carto-Server erlauben das; Kartendaten sind weiterhin OSM.
    karte = folium.Map(location=[float(np.mean(lat)), float(np.mean(lon))],
                       zoom_start=16, control_scale=True,
                       tiles="CartoDB positron")
    karte.add_to(figur)

    obergrenze = max(max(werte, default=0.0), 1.0)
    skala = cm.linear.YlOrRd_09.scale(0.0, obergrenze)
    skala.caption = "max. Bremsdruck im Abschnitt [bar]"
    skala.add_to(karte)

    for i in range(len(t) - 1):
        if t[i + 1] - t[i] > GNSS_LUECKE_MAX_S:
            continue  # lange Empfangsluecke: bewusst nicht verbinden
        folium.PolyLine(
            [(lat[i], lon[i]), (lat[i + 1], lon[i + 1])],
            color=skala(werte[i]), weight=5, opacity=0.9,
            dash_array="5,10" if (interp[i] or interp[i + 1]) else None,
            tooltip=f"t={t[i]:.0f} s, v={spur['v_km_h'].iloc[i]:.1f} km/h, "
                    f"p_max={werte[i]:.1f} bar",
        ).add_to(karte)

    folium.Marker([lat[0], lon[0]], tooltip="Start",
                  icon=folium.Icon(color="green", icon="play")).add_to(karte)
    folium.Marker([lat[-1], lon[-1]], tooltip="Ende",
                  icon=folium.Icon(color="red", icon="stop")).add_to(karte)

    if not ereignisse.empty:
        for nr, z in enumerate(ereignisse.itertuples(index=False), start=1):
            if not z.fix:
                continue  # ohne Fix keine brauchbare Position
            folium.CircleMarker(
                [z.lat_deg, z.lon_deg], radius=7, color="black",
                fill=True, fill_color="#d62728", fill_opacity=0.9,
                tooltip=(f"Bremsereignis {nr}: {z.dauer_s:.1f} s, "
                         f"vorne {z.p_max_vorne_bar:.1f} bar / "
                         f"hinten {z.p_max_hinten_bar:.1f} bar, "
                         f"{z.v_vorher_kmh:.0f} -> {z.v_nachher_kmh:.0f} km/h"),
            ).add_to(karte)

    return figur._repr_html_()


def _tabelle(ereignisse: pd.DataFrame) -> str:
    if ereignisse.empty:
        return "<p>Keine Bremsereignisse erkannt.</p>"
    anzeige = ereignisse.drop(columns=["lat_deg", "lon_deg", "fix"]).copy()
    anzeige.columns = ["Start [s]", "Dauer [s]", "p max vorne [bar]",
                       "p max hinten [bar]", "v vorher [km/h]",
                       "v nachher [km/h]", "Verzoegerung [m/s^2]"]
    anzeige.index = range(1, len(anzeige) + 1)
    return anzeige.to_html(float_format=lambda x: f"{x:.1f}", border=0)


def erzeuge_report(df: pd.DataFrame, spur: pd.DataFrame,
                   ereignisse: pd.DataFrame, pfad: Path) -> None:
    teile = [
        "<meta charset='utf-8'>",
        f"<title>Report {html.escape(pfad.name)}</title>",
        "<style>body{font-family:sans-serif;margin:20px;max-width:1200px}"
        "h1,h2{color:#333} table{border-collapse:collapse}"
        "th,td{padding:4px 10px;text-align:right;border-bottom:1px solid #ddd}"
        "</style>",
        f"<h1>Messfahrt-Report: {html.escape(pfad.name)}</h1>",
        "<h2>Zeitverlaeufe (zoombar)</h2>",
        _zeitreihen(df),
        "<h2>Bremsereignisse</h2>",
        _tabelle(ereignisse),
    ]

    karte = _karte(df, spur, ereignisse)
    if karte is not None:
        teile += ["<h2>Strecke (Farbe = Bremsdruck)</h2>",
                  "<p>Gestrichelt = interpolierte GNSS-Luecke. "
                  "Kartenkacheln brauchen Internet.</p>", karte]
    else:
        teile += ["<h2>Strecke</h2><p>Kein GNSS-Fix in dieser Messung.</p>"]

    ziel = pfad.with_name(pfad.stem + "_report.html")
    ziel.write_text("\n".join(teile), encoding="utf-8")
    print(f"Report gespeichert: {ziel}")
