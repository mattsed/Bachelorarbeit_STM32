#include "comms/ble_bluenrg.h"

static bool ble_bluenrg_ready = false;

app_status_t ble_bluenrg_init(void)
{
  ble_bluenrg_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t ble_bluenrg_poll(void)
{
  return APP_STATUS_NOT_READY;
}

app_status_t ble_bluenrg_send_sample(const app_sample_t *sample)
{
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool ble_bluenrg_is_ready(void)
{
  return ble_bluenrg_ready;
}