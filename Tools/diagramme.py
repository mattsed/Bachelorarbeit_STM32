"""
Statische Diagramme (matplotlib): Zeitverlaeufe und GPS-Spur als PNG.

Diese Plots sind der Offline-Fallback und die druckbare Variante fuer die
Arbeit; die interaktive Ansicht (zoombar, Karte) erzeugt report.py.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from konstanten import GNSS_LUECKE_MAX_S


def plots(df: pd.DataFrame, basis: Path) -> None:
    """Zeichnet die fuenf Zeitverlaeufe (IMU-Beschleunigung, Drehrate,
    400g-Sensor, Bremsdruck, GNSS-Geschwindigkeit) untereinander mit
    gemeinsamer Zeitachse und speichert sie als <name>_plots.png."""
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
    """Zeichnet die GPS-Spur in lokalen Ost/Nord-Metern und speichert sie
    als <name>_track.png.

    Darstellung: echte Segmente farbig nach Geschwindigkeit (viridis),
    interpolierte Ersatzpunkte grau gestrichelt, lange Empfangsluecken
    werden gar nicht verbunden (sichtbarer Bruch in der Spur)."""
    if spur is None:
        return  # kein Fix oder keine Bewegung -> keine sinnvolle Spur

    # Spur in Einzelsegmente zwischen aufeinanderfolgenden Punkten zerlegen
    # und nach echt/interpoliert sortieren.
    p = spur[["ost_m", "nord_m"]].to_numpy()
    t = spur["t_s"].to_numpy()
    v = spur["v_km_h"].to_numpy()
    interp = spur["interp"].to_numpy()

    seg_echt, seg_interp, v_echt = [], [], []
    for i in range(len(p) - 1):
        if t[i + 1] - t[i] > GNSS_LUECKE_MAX_S:
            continue  # lange Luecke: bewusst nicht verbinden
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
