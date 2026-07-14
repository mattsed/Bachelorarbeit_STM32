#ifndef ACC_ADXL373_H
#define ACC_ADXL373_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialisiert den ADXL373 400g-Beschleunigungssensor auf SPI3. */
app_status_t acc_adxl373_init(void);

/* Liest die 400g-Beschleunigungsrohdaten des ADXL373. */
app_status_t acc_adxl373_read(acc_400g_data_t *data);

/* Meldet, ob der ADXL373 erkannt und konfiguriert ist. */
bool acc_adxl373_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* ACC_ADXL373_H */
