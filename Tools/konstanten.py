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

# Nullpunkt je Fahrt und Kanal (siehe daten.nullpunkt_bar): Der PSS-140 ist
# ein ABSOLUTdrucksensor und zeigt im Ruhezustand den Umgebungsluftdruck
# (auf 600 m rund 0,94 bar). Dazu kommt seine Toleranz von +/-1 % vom
# Endwert, bei 140 bar Messbereich also +/-1,4 bar -- gemessen wurden am
# 16.08.2026 im Stand 1,1 bar vorne und 0,4 bar hinten. Ein fester Abzug
# griffe zu kurz, deshalb wird der Ruhepegel aus den Daten geschaetzt.
# Fenster fuer den gleitenden Median VOR der Perzentilbildung: eine halbe
# Sekunde daempft das Einzelwandlungs-Rauschen (~ +/-1,7 bar), ist aber kurz
# genug, dass eine Bremsung den Ruhepegel nicht anhebt.
ABTASTRATE_HZ = 50.0
NULLPUNKT_FENSTER_S = 0.5
NULLPUNKT_PERZENTIL = 5.0

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

# ------------------------------------------------------------ Sensorfusion
# Lagefilter (Madgwick): beta gewichtet die Accelerometer-Korrektur gegen
# die Gyro-Integration. Klein = traege, aber vibrationsunempfindlich;
# gross = schnell, aber jede Stoerbeschleunigung verbiegt die Lage.
# 0,05 ist ein ueblicher Wert fuer Fahrzeuganwendungen mit Vibration.
MADGWICK_BETA = 0.05

# Welche IMU-Achse in Fahrtrichtung zeigt und mit welchem Vorzeichen
# (+1 = positive Achsrichtung nach vorn). TODO: erst mit einer echten
# Fahrmessung kalibrierbar -- beim Anfahren muss die neigungsbereinigte
# Laengsbeschleunigung positiv werden, beim Bremsen negativ.
FAHRT_ACHSE = "x"
FAHRT_VORZEICHEN = +1.0

# Geschwindigkeits-Kalman (1D): Prozessrauschen beschreibt, wie stark die
# per IMU praedizierte Geschwindigkeit pro Schritt vom wahren Wert
# abweichen kann (Achsfehler, Restneigung, Vibrationen); Messrauschen die
# Unsicherheit der GNSS-Geschwindigkeit (Doppler-basiert, im Stand ~0,5 m/s
# Rauschen, siehe STILLSTAND_SCHWELLE_M_S). Das Verhaeltnis der beiden
# bestimmt das Kalman-Gain, also wie stark jedes GNSS-Update die
# IMU-Praediktion zurueckzieht.
# TODO Solange FAHRT_ACHSE unkalibriert ist, ist die IMU-Praediktion wenig
# vertrauenswuerdig -> Prozessrauschen bewusst gross (2,5), damit das GNSS
# dominiert. Nach der Achskalibrierung mit einer echten Fahrmessung kann
# der Wert Richtung ~0,5..1,0 gesenkt werden (IMU bekommt mehr Gewicht,
# Profil zwischen den GNSS-Stuetzstellen wird informativer).
KALMAN_SIGMA_A_M_S2 = 2.5       # Standardabw. des Beschleunigungsfehlers
KALMAN_SIGMA_GNSS_M_S = 0.5     # Standardabw. der GNSS-Geschwindigkeit
G_M_S2 = 9.81                   # Erdbeschleunigung fuer g -> m/s^2
