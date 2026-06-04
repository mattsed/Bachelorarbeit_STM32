#include "sensors/imu_lsm6dso.h"

static bool imu_lsm6dso_ready = false;

app_status_t imu_lsm6dso_init(void)
{
  imu_lsm6dso_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t imu_lsm6dso_read(imu_data_t *data)
{
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool imu_lsm6dso_is_ready(void)
{
  return imu_lsm6dso_ready;
}