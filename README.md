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

