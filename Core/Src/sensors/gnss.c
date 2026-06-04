#include "sensors/gnss.h"

static bool gnss_ready = false;

app_status_t gnss_init(void)
{
  gnss_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t gnss_poll(void)
{
  return APP_STATUS_NOT_READY;
}

app_status_t gnss_read(gnss_data_t *data)
{
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool gnss_is_ready(void)
{
  return gnss_ready;
}