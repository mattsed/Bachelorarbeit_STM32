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

#ifdef __cplusplus
}
#endif

#endif /* IMU_LSM6DSO_H */
