"""
Zentrale Konstanten der Datenlogger-Auswertung.

Alle Umrechnungsfaktoren und Schwellwerte an einer Stelle, damit sie
zwischen den Modulen (daten, gnss, bremsen, konsole, diagramme, report)
garantiert konsistent sind. Die Werte muessen zur Firmware-Konfiguration
passen -- die jeweilige Quelle steht als Kommentar dabei.
"""

# ------------------------------------------------------------ Sensor-Skalen
# LSM6DSO-IMU, konfiguriert in imu_lsm6dso.c: +/-16 g und +/-2000 dps.
IMU_ACCEL_G_PER_LSB = 0.488e-3      # Datenblatt-Empfindlichkeit bei +/-16 g
IMU_GYRO_DPS_PER_LSB = 70e-3        # Datenblatt-Empfindlichkeit bei +/-2000 dps

# ADXL373 400g-Beschleunigungssensor: feste Aufloesung 200 mg/LSB.
ACC400_G_PER_LSB = 0.2

# ---------------------------------------------------------- Bremsdruck-Kette
# 12-Bit-ADC des STM32 (3,3-V-Referenz) hinter dem Spannungsteiler 15k/33k,
# davor der Drucksensor Bosch PSS-140 (0,5 V Offset, 28,571 mV/bar).
ADC_VREF = 3.3
ADC_MAX = 4095.0
TEILER = 33.0 / 48.0                # Teilerverhaeltnis 33k / (15k + 33k)
PSS_OFFSET_V = 0.5
PSS_V_PER_BAR = 0.028571

# Plausibilitaetsfenster wie in der Firmware (brake_pressure.c): Der PSS-140
# liefert gesund immer 0,5..4,725 V. Deutlich darunter = Kabelbruch/keine
# Versorgung, deutlich darueber = Kurzschluss. Solche Werte werden NaN,
# damit ein Kabelbruch nicht als "137 bar Vollbremsung" in den Plots landet.
PSS_FEHLER_MIN_V = 0.35
PSS_FEHLER_MAX_V = 4.75

# ------------------------------------------------------------------- GNSS
# GNSS-Rauschen im Stand: unterhalb dieser Geschwindigkeit gilt "steht"
# (empirisch ermittelt, siehe Protokoll_Inbetriebnahme.txt).
STILLSTAND_SCHWELLE_M_S = 0.5

# GNSS-Bereinigung: Punkte, die eine unmoegliche Geschwindigkeit implizieren
# (Mehrwegempfang, kurzzeitig schlechte Satellitengeometrie), fliegen raus.
# 40 m/s = 144 km/h -- beim Downhill physikalisch nicht erreichbar.
GNSS_MAX_SPRUNG_M_S = 40.0
GNSS_LUECKE_MIN_S = 1.5     # normaler Update-Abstand ist 1 s; ab hier "Luecke"
GNSS_LUECKE_MAX_S = 3.0     # bis hierhin wird linear interpoliert, danach Bruch

# ---------------------------------------------------- Bremsereignis-Erkennung
# Schwellwert mit Hysterese (Beginn ueber 2 bar, Ende erst unter 1,5 bar --
# verhindert Flattern um eine einzelne Schwelle). Die Mindestdauer verwirft
# einzelne Rauschspitzen.
BREMS_START_BAR = 2.0
BREMS_ENDE_BAR = 1.5
BREMS_MIN_DAUER_S = 0.2
