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

#ifdef __cplusplus
}
#endif

#endif /* GNSS_H */
