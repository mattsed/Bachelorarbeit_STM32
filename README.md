# Datenlogger zur Bremsanalyse am Mountainbike

Firmware und Auswertesoftware eines Datenloggers, der Bremsdruck, Fahrdynamik
und Position synchron aufzeichnet. Entstanden als Bachelorarbeit im Studiengang
Mechatronik, Design & Innovation am MCI Innsbruck.

Das System nimmt an beiden Bremsen den Hydraulikdruck auf, dazu Beschleunigung
und Drehrate einer IMU, Stoßbeschleunigungen bis 400 g sowie Position und
Geschwindigkeit über GNSS. Alle Kanäle werden mit einem gemeinsamen Zeitstempel
versehen und im 20-ms-Raster als CSV-Zeile auf eine microSD-Karte geschrieben.
Die Auswertung erfolgt nach der Fahrt am Rechner.

## Hardware

Aufgebaut aus Evaluation-Boards, verbunden über eine Lochrasterplatine.

| Funktion | Bauteil | Anbindung |
|---|---|---|
| Mikrocontroller | NUCLEO-H563ZI (STM32H563ZI, 250 MHz) | – |
| Bremsdruck vorne | Bosch PSS-140, 0–140 bar | ADC1, PA0 |
| Bremsdruck hinten | Bosch PSS-140, 0–140 bar | ADC1, PA3 |
| IMU | STEVAL-MKI196V1 (LSM6DSO) | SPI3, CS PD5 |
| Stoßbeschleunigung | EVAL-ADXL373Z (±400 g) | SPI3, CS PD4 |
| GNSS | X-NUCLEO-LIV4A1 (Teseo-LIV4F) | LPUART1, RX PB7 / TX PB6 |
| Speicher | SparkFun BOB-00544, microSD | SPI5, CS PB12 |
| Versorgung | USB-Powerbank 5 V | – |

Sensoren und Speicherkarte liegen bewusst auf getrennten SPI-Bussen, damit ein
Schreibzugriff auf die Karte das Auslesen der Sensoren nicht blockiert.

Die PSS-140 liefern 0,5–4,5 V bei 5 V Versorgung. Ein Spannungsteiler aus
15 kΩ und 33 kΩ bringt das auf den 3,3-V-Bereich des Analog-Digital-Umsetzers.


## Aufbau des Repositorys

```
Core/Inc, Core/Src      Firmware
  app/                  Ablaufsteuerung, Messzyklus, Zustandsautomat
  board/                Zuordnung der von CubeMX erzeugten HAL-Handles
  common/               gemeinsame Statuscodes und Datenstrukturen
  sensors/              Treiber: GNSS, Bremsdruck, LSM6DSO, ADXL373
  storage/              microSD-Logger und FatFS
Drivers/                HAL und BSP von STMicroelectronics
Tools/                  Auswertung in Python
```

Der Rest (`.ioc`, `.cproject`, Linker-Skripte, `Debug/`) stammt aus
STM32CubeIDE beziehungsweise dem Build.



Die Konsolenausgabe der Firmware liegt auf dem virtuellen COM-Port des
ST-LINK, 115200 Baud 8N1.

## Bedienung

1. Karte einstecken, Powerbank anschließen. Die Firmware initialisiert alle
   Sensoren und geht in den Zustand **bereit**.
2. Die gelbe LED (LD2) zeigt den GNSS-Fix an: leuchtet sie, liegt eine gültige
   Position vor. Vor dem Start abwarten, sonst bleiben Position, Geschwindigkeit
   und UTC-Zeit der ersten Zeilen auf null.
3. **B1 (blauer Knopf)** startet die Aufzeichnung. Es wird eine neue Datei
   `LOG_nnn.CSV` angelegt.
4. **RESET (schwarzer Knopf)** beendet die Aufzeichnung und bootet zurück in
   den Bereit-Zustand.

Ein Watchdog setzt das System nach rund 4 s ohne Lebenszeichen zurück.

## Format der Messdatei

Semikolongetrennt, eine Zeile je Messzeitpunkt, 50 Hz. Vor der Kopfzeile steht
als Kommentar der beim Start ermittelte Gyroskop-Nullpunkt:

```
# gyro_bias;11;10;-5
t_ms;fix;lat_e7;lon_e7;v_mm_s;utc_ms;p_vorne_raw;p_hinten_raw;
imu_ax;imu_ay;imu_az;imu_gx;imu_gy;imu_gz;acc400_x;acc400_y;acc400_z
```

| Feld | Einheit | Umrechnung |
|---|---|---|
| `t_ms` | ms seit Systemstart | – |
| `fix` | 0 oder 1 | Gültigkeitsflag des GNSS-Empfängers |
| `lat_e7`, `lon_e7` | Grad · 10⁷ | – |
| `v_mm_s` | mm/s | – |
| `utc_ms` | ms seit Mitternacht UTC | – |
| `p_vorne_raw`, `p_hinten_raw` | ADC-Rohwert, 12 bit | siehe unten |
| `imu_a*` | LSB | · 0,488 mg (±16 g) |
| `imu_g*` | LSB | · 70 mdps (±2000 dps) |
| `acc400_*` | LSB | · 200 mg (±400 g) |

Der Bremsdruck ergibt sich aus dem Rohwert über

```
U_Pin    = N / 4095 · 3,3 V
U_Sensor = U_Pin / (33 / 48)
p        = (U_Sensor − 0,5 V) / 28,571 mV/bar
```

Der PSS-140 misst **absolut**. Im drucklosen Zustand zeigt er den
Umgebungsluftdruck an, nicht null. Die Auswertung bestimmt den Ruhewert pro
Fahrt und Kanal aus den Daten selbst und zieht ihn ab; damit fallen Luftdruck,
Höhenlage und der Offsetfehler des Sensors gemeinsam heraus.

## Auswertung

```bash
cd Tools
pip install numpy pandas matplotlib scipy
pip install plotly folium branca      # optional, für den HTML-Report

python auswertung.py ../../Messungen/LOG_047.CSV
```

Erzeugt eine Zusammenfassung auf der Konsole, eine Tabelle der erkannten
Bremsereignisse, Zeitverlaufsdiagramme, die GNSS-Spur als Karte sowie einen
interaktiven HTML-Report.

| Modul | Aufgabe |
|---|---|
| `konstanten.py` | alle Umrechnungsfaktoren und Schwellwerte |
| `daten.py` | CSV laden, Rohwerte in physikalische Einheiten |
| `gnss.py` | 1-Hz-Spur extrahieren, Ausreißer und Lücken behandeln |
| `fusion.py` | Lagefilter (Madgwick) und Geschwindigkeits-Kalman |
| `bremsen.py` | Bremsereignisse über Hysterese erkennen |
| `kalibrierung.py` | Fahrtrichtungsachse aus einer Kalibrierfahrt bestimmen |
| `diagramme.py`, `report.py` | Diagramme und HTML-Report |

`konstanten.py` muss zur Firmware-Konfiguration passen. Wird an der Firmware
etwas geändert, ist die Datei nachzuziehen.

### Kalibrierung der Fahrtrichtungsachse

Das Gehäuse sitzt an der Flaschenhalteraufnahme und damit schräg. Keine
Sensorachse zeigt in Fahrtrichtung. `kalibrierung.py` bestimmt aus einer Fahrt
mit wiederholtem Beschleunigen und Bremsen auf ebener Strecke, welche Achse am
besten mit der GNSS-Beschleunigung übereinstimmt, und liefert Vorzeichen und
Maßstab. Die Werte werden in `konstanten.py` eingetragen.

## Lizenz

MIT für den in dieser Arbeit entstandenen Code (`Core/Inc`, `Core/Src` ohne die
von CubeMX erzeugten Dateien, sowie `Tools/`).

`Drivers/` enthält HAL und BSP von STMicroelectronics unter deren eigenen
Lizenzbedingungen, `Core/Src/storage/ff.c` FatFS von ChaN unter der
FatFS-Lizenz. Beide sind unverändert übernommen.

## Zitierung

Sedlaczek, M. (2026). *Entwicklung eines Datenloggers zur synchronen Erfassung
von Bremsdruck, Fahrdynamik und Position im Mountainbike-Downhill.*
Bachelorarbeit, MCI Innsbruck. Zenodo. https://doi.org/TODO
