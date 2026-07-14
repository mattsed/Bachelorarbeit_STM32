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

/* Meldet, ob der Logger bereit ist, Daten dauerhaft zu speichern. */
bool storage_logger_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_LOGGER_H */
