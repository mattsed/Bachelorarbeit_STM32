#ifndef STORAGE_LOGGER_H
#define STORAGE_LOGGER_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bereitet microSD/FatFS fuer das spaetere Schreiben von Messdaten vor. */
app_status_t storage_logger_init(void);

/* Schreibt einen kompletten Messdatensatz auf die microSD. */
app_status_t storage_logger_write_sample(const app_sample_t *sample);

/* Beendet die laufende Aufzeichnung sauber (Datei schliessen). */
app_status_t storage_logger_stop(void);

/* Startet eine neue Logdatei (naechste freie LOG_nnn.CSV). */
app_status_t storage_logger_start(void);

/* Versucht nach einem SD-Fehler im Betrieb (Karte gezogen, Schreibfehler)
 * einen kompletten Neuaufbau: Karte neu initialisieren, Dateisystem neu
 * mounten, neue Logdatei anlegen. */
app_status_t storage_logger_recover(void);

/* Meldet, ob gerade eine Aufzeichnung laeuft. */
bool storage_logger_is_ready(void);

/* Meldet, ob die Karte gemountet und der Logger startbereit ist
 * (unabhaengig davon, ob gerade aufgezeichnet wird). */
bool storage_logger_is_available(void);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_LOGGER_H */
