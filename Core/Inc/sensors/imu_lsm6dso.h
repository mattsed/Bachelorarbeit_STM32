#ifndef IMU_LSM6DSO_H
#define IMU_LSM6DSO_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialisiert die LSM6DSO-IMU auf dem gemeinsamen SPI3-Sensorbus. */
app_status_t imu_lsm6dso_init(void);

/* Liest Beschleunigungs- und Gyro-Rohwerte der LSM6DSO-IMU. */
app_status_t imu_lsm6dso_read(imu_data_t *data);

/* Meldet, ob die LSM6DSO-IMU erkannt und konfiguriert ist. */
bool imu_lsm6dso_is_ready(void);

/* Liefert den beim Start (im Stand) gemessenen Gyro-Nullpunktfehler in
 * Rohwerten (LSB). Rueckgabe false, falls keine Kalibrierung vorliegt. */
bool imu_lsm6dso_get_gyro_bias(int16_t *gx, int16_t *gy, int16_t *gz);

/* Schaltet die Bias-Messung in imu_lsm6dso_init() ab. Muss VOR dem Init
 * aufgerufen werden.
 *
 * WARUM: Die Messung mittelt rund 2 s lang die Drehraten und setzt dabei
 * voraus, dass das Rad still steht. Nach einem Watchdog-Reset waehrend der
 * Fahrt trifft das nicht zu -- die gemessene "Nullpunktabweichung" wuerde
 * dann echte Drehung enthalten und in der Auswertung von allen Messwerten
 * abgezogen. Ohne Messung bleibt der Bias ungueltig, die Kopfzeile
 * "# gyro_bias" entfaellt, und die Auswertung weiss, dass fuer diese Datei
 * kein Bias vorliegt. */
void imu_lsm6dso_set_bias_calibration(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LSM6DSO_H */
