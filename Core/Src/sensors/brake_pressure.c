#include "sensors/brake_pressure.h"

static bool brake_pressure_ready = false;

app_status_t brake_pressure_init(void)
{
  brake_pressure_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t brake_pressure_read(brake_pressure_data_t *data)
{
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool brake_pressure_is_ready(void)
{
  return brake_pressure_ready;
}