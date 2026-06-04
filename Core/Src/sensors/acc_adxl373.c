#include "sensors/acc_adxl373.h"

static bool acc_adxl373_ready = false;

app_status_t acc_adxl373_init(void)
{
  acc_adxl373_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t acc_adxl373_read(acc_400g_data_t *data)
{
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool acc_adxl373_is_ready(void)
{
  return acc_adxl373_ready;
}