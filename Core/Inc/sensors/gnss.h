#ifndef GNSS_H
#define GNSS_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bereitet den GNSS-Empfang ueber die vorgesehene UART-Schnittstelle vor. */
app_status_t gnss_init(void);

/* Verarbeitet empfangene GNSS-Daten im Hintergrund, z. B. NMEA-Zeichen. */
app_status_t gnss_poll(void);

/* Gibt die zuletzt gueltig gelesenen GNSS-Daten an die app-Schicht weiter. */
app_status_t gnss_read(gnss_data_t *data);

/* Meldet, ob das GNSS-Modul initialisiert ist und Daten liefern kann. */
bool gnss_is_ready(void);

/* Meldet, ob aktuell eine gueltige Position vorliegt (RMC-Status 'A').
 * Billig genug, um in jeder Schleifenrunde abgefragt zu werden. */
bool gnss_has_fix(void);

/* Sendet einen proprietaeren NMEA-Befehl an das Modul. Uebergeben wird nur
 * der Rumpf zwischen '$' und '*', z. B. "PSTMSETPAR,1303,0.2,0" fuer 5 Hz;
 * Pruefsumme und Zeilenende ergaenzt die Funktion selbst. */
app_status_t gnss_send_command(const char *body);

/* Fragt die Firmware-Version ab und gibt die Antwort per printf aus.
 * Blockiert dabei rund 2 s -- nur aus app_init() aufrufen, also bevor der
 * Watchdog scharf geschaltet wird. */
app_status_t gnss_query_version(void);

/* Bring-up des UART-Wegs zum Teseo (LPUART1, PB6/PB7 ueber die Arduino-
 * Pins D0/D1): sucht die Baudrate des Moduls und fragt dort die
 * Firmware-Version ab. Blockiert mehrere Sekunden -- nur aus app_init(). */
app_status_t gnss_uart_bringup(void);

/* Diagnose: liest fuer die angegebene Dauer den rohen NMEA-Strom mit, gibt
 * jede vollstaendige Zeile auf der Konsole aus und zaehlt am Ende die
 * Satzarten. Blockiert entsprechend lange -- nur aus app_init() aufrufen,
 * solange der Watchdog noch nicht scharf ist. */
app_status_t gnss_dump_stream(uint32_t duration_ms);

/* Liefert das zuletzt empfangene UTC-Datum und die UTC-Zeit (fuer die
 * Datei-Zeitstempel auf der microSD). false = noch kein Datum empfangen. */
bool gnss_get_utc_datetime(uint16_t *year, uint8_t *month, uint8_t *day,
                           uint8_t *hour, uint8_t *minute, uint8_t *second);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_H */
