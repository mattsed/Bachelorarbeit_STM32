# Bachelorarbeit_STM32

STM32CubeIDE-Projekt fuer ein Embedded-Systems-Datenmodul auf Basis des NUCLEO-H563ZI.

## Zielsystem

Das Modul soll spaeter GPS/GNSS-Daten, Bremsdruckdaten, IMU-Daten und 400g-Beschleunigungsdaten erfassen, auf microSD speichern und per BLE versenden.

## Aktuelle Modulstruktur

Die erste Projektstruktur trennt CubeMX-generierten Code von der spaeteren Applikationslogik. Derzeit enthalten die Module bewusst nur Schnittstellen und Stubs, noch keine produktive Sensor-, Logging- oder BLE-Logik.

- `Core/Inc/app` und `Core/Src/app`: zentrale Applikationsschicht mit `app_init()` und `app_run()`.
- `Core/Inc/board` und `Core/Src/board`: zentrale Zuordnung der von CubeMX initialisierten HAL-Handles.
- `Core/Inc/common`: gemeinsame Statuscodes und Datenstrukturen.
- `Core/Inc/sensors` und `Core/Src/sensors`: Platzhalter fuer GNSS, Bremsdruck, LSM6DSO-IMU und ADXL373.
- `Core/Inc/storage` und `Core/Src/storage`: Platzhalter fuer microSD/FatFS-Logging.
- `Core/Inc/comms` und `Core/Src/comms`: Platzhalter fuer BLE ueber BlueNRG.

## Aktueller Entwicklungsstand

Das Hardwaremodul ist noch nicht vollstaendig aufgebaut. Deshalb werden aktuell
keine praxisnahen Sensor-, microSD- oder BLE-Tests vorausgesetzt. Der Softwarestand
bereitet die Architektur, Schnittstellen, Kommentare und das Hardware-Mapping vor.
Echte Treiberlogik soll erst schrittweise nach dem Hardware-Bring-up ergaenzt
werden.

## Hardware-Mapping

- X-NUCLEO-LIV4A1 GNSS: LPUART1, PB6/PB7, NMEA/Command-UART geplant.
- X-NUCLEO-BNRG2A1 BLE: SPI1 mit BLE_CS, BLE_IRQ und BLE_RESET.
- BOB-00544 microSD: SPI5 mit microSD_CS auf PB12.
- Bosch PSS-140 Bremsdrucksensoren: ADC1 auf PA0 und PA3.
- EVAL-ADXL373Z 400g-Beschleunigungssensor: SPI3.
- STEVAL-MKI196V1 IMU: LSM6DSO auf SPI3.

## SPI-Zuordnung

- SPI1: X-NUCLEO-BNRG2A1 BLE.
- SPI3: STEVAL-MKI196V1 IMU und EVAL-ADXL373Z 400g-Beschleunigungssensor.
- SPI5: BOB-00544 microSD.
- SPI2: wird fuer diese Anwendung normalerweise nicht verwendet.

## Geplanter Datenfluss

Die `app`-Schicht steuert spaeter den Messablauf. Sie sammelt die Daten der
einzelnen Module in einem gemeinsamen `app_sample_t` Messdatensatz. Dieser
Datensatz enthaelt Zeitstempel, GNSS-Daten, Bremsdruckdaten, IMU-Daten und
ADXL373-400g-Beschleunigungsdaten.

Der geplante Ablauf ist:

1. GNSS und BLE im Hintergrund pollen.
2. Einen neuen `app_sample_t` Messdatensatz anlegen.
3. Bremsdruck, IMU, ADXL373 und die zuletzt verfuegbaren GNSS-Daten einlesen.
4. Die Rueckgabewerte der Module ueber `app_status_t` bewerten.
5. Den Messdatensatz auf der microSD speichern.
6. Den gleichen Messdatensatz optional per BLE versenden.

Die microSD ist der primaere Speicher fuer die vollstaendige Messfahrt. BLE ist
als Live- oder Diagnosekanal vorgesehen. Wenn BLE nicht verbunden oder nicht
bereit ist, sollen Messdaten trotzdem weiter auf die microSD geschrieben werden.

## Chip-Selects und Steuerpins

- BLE_CS: PC0, BLE_IRQ: PA6, BLE_RESET: PG12.
- microSD_CS: PB12.
- ACC_CS: PD4 fuer ADXL373.
- IMU_CS: PD5 fuer LSM6DSO.

## Naechste Bring-up-Reihenfolge

1. SPI3 fuer ADXL373 und LSM6DSO mit getrennten Chip-Selects pruefen.
2. microSD einzeln in Betrieb nehmen: SPI5, Chip-Select, FatFS-Mount, Testdatei schreiben.
3. Bremsdruckkanaele kalibrieren: ADC-Rohwerte lesen, Offset/Skalierung fuer Bosch PSS-140 eintragen.
4. GNSS UART in Betrieb nehmen: NMEA-Empfang puffern und Fix/Position/Zeit parsen.
5. IMU und ADXL373 je einzeln per WHOAMI/Registertest pruefen.
6. BLE erst nach stabilem Logger anbinden: BlueNRG SPI/IRQ/Reset, danach Datenpakete streamen.

## Build-Hinweis

Wenn das Projekt in STM32CubeIDE bereits geoeffnet ist, nach den neuen Dateien einmal das Projekt aktualisieren (`F5`/Refresh) und dann den Debug-Build starten. Die `.cproject` fuehrt `Core` als Source-Folder, daher sollten die neuen Unterordner beim CubeIDE-Build mitgebaut werden.
