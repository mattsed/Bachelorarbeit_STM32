#ifndef IMU_LSM6DSO_H
#define IMU_LSM6DSO_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t imu_lsm6dso_init(void);
app_status_t imu_lsm6dso_read(imu_data_t *data);
bool imu_lsm6dso_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LSM6DSO_H */