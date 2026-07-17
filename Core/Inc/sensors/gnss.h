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

/* Liefert das zuletzt empfangene UTC-Datum und die UTC-Zeit (fuer die
 * Datei-Zeitstempel auf der microSD). false = noch kein Datum empfangen. */
bool gnss_get_utc_datetime(uint16_t *year, uint8_t *month, uint8_t *day,
                           uint8_t *hour, uint8_t *minute, uint8_t *second);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_H */
