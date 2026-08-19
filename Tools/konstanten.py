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
# (+1 = positive Achsrichtung nach vorn).
#
# Kalibriert am 19.08.2026 mit LOG_045 (253 s, 130 auswertbare Intervalle,
# kalibrierung.py): Achse x, r = +0,802. Die Achsen x und z korrelieren fast
# gleich stark (0,802 gegen -0,800), x sagt die GNSS-Referenz aber besser
# vorher -- Restfehler 0,64 gegen 0,76 m/s^2. Eine gemeinsame Projektion
# aller drei Achsen bringt nur 0,63 m/s^2, eine Achse genuegt also.
FAHRT_ACHSE = "x"
FAHRT_VORZEICHEN = +1.0

# Massstab fuer die Einbaulage. Das Board liegt flach am Unterrohr, der
# Lagefilter meldet im Mittel Pitch -62,5 Grad. Die Fahrtrichtungsachse
# sieht davon nur cos(62,5 Grad) = 0,46 der wahren Laengsbeschleunigung.
# Der Wert stammt nicht aus dieser Geometrie, sondern aus der Regression
# gegen die GNSS-Referenz (Steigung 2,50) -- er faengt damit auch eine
# Restverdrehung und einen Rest an nicht herausgerechneter Schwerkraft mit
# ab. Ohne diesen Faktor waere a_laengs dauerhaft rund 60 % zu klein.
FAHRT_SKALIERUNG = 2.50

# Geschwindigkeits-Kalman (1D): Prozessrauschen beschreibt, wie stark die
# per IMU praedizierte Geschwindigkeit pro Schritt vom wahren Wert
# abweichen kann (Achsfehler, Restneigung, Vibrationen); Messrauschen die
# Unsicherheit der GNSS-Geschwindigkeit (Doppler-basiert, im Stand ~0,5 m/s
# Rauschen, siehe STILLSTAND_SCHWELLE_M_S). Das Verhaeltnis der beiden
# bestimmt das Kalman-Gain, also wie stark jedes GNSS-Update die
# IMU-Praediktion zurueckzieht.
# BLEIBT bei 2,5 -- die urspruengliche Erwartung, den Wert nach der
# Achskalibrierung auf 0,5..1,0 senken zu koennen, ist widerlegt.
#
# Geprueft an LOG_045 mit Rueckhaltetest: jedes zweite GNSS-Update wurde dem
# Filter vorenthalten, sodass er 2 s allein mit der IMU ueberbruecken muss,
# und gegen die zurueckgehaltenen Werte geprueft (126 Pruefpunkte). Das
# vermeidet den Zirkelschluss, die Fusion an genau den Stuetzstellen zu
# bewerten, die sie selbst einarbeitet.
#
#   ohne IMU (letzten GNSS-Wert halten)   4,75 km/h RMS
#   sigma_a = 0,65                        6,81
#   sigma_a = 2,5                         3,86
#   sigma_a = 10                          3,12
#
# Die IMU verbessert das Ergebnis also deutlich, aber nur bei grossem
# Prozessrauschen: Ihr Verlauf zwischen den Stuetzstellen ist brauchbar, ihr
# absoluter Pegel driftet. Ein fester Bias erklaert das nicht vollstaendig
# (abgezogen bleiben bei sigma_a = 0,65 immer noch 5,35 km/h) -- der Fehler
# ist zeitveraenderlich. Sauber waere, den Bias als zweiten Filterzustand
# mitzuschaetzen statt ihn ueber ein aufgeblaehtes Prozessrauschen
# abzufangen; 10 statt 2,5 brachte nur 0,7 km/h und waere als
# "Beschleunigungsfehler von 1 g" nicht mehr physikalisch begruendbar.
KALMAN_SIGMA_A_M_S2 = 2.5       # Standardabw. des Beschleunigungsfehlers
KALMAN_SIGMA_GNSS_M_S = 0.5     # Standardabw. der GNSS-Geschwindigkeit
G_M_S2 = 9.81                   # Erdbeschleunigung fuer g -> m/s^2
