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

/* Setzt die softwareseitige Spitzenwertverfolgung zurueck (Aufruf bei
 * Aufzeichnungsstart). Ersetzt die MAXPEAK-Hardwareregister des Sensors,
 * die auf realer Hardware zuverlaessig 0 liefern (siehe acc_adxl373.c). */
void acc_adxl373_reset_peak_tracking(void);

/* Liefert die groessten seit dem letzten Reset beobachteten Betraege je
 * Achse (mit Originalvorzeichen). Werte in Roheinheiten (1 LSB = 200 mg),
 * aktualisiert bei jedem acc_adxl373_read()-Aufruf. */
void acc_adxl373_get_peak_tracking(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif /* ACC_ADXL373_H */
