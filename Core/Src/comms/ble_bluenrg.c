#include "comms/ble_bluenrg.h"

/* Wird true, sobald das BlueNRG-Modul initialisiert und bereit zum Senden ist. */
static bool ble_bluenrg_ready = false;

app_status_t ble_bluenrg_init(void)
{
  /* Spaeter: BlueNRG ueber SPI1, IRQ und Reset initialisieren. */
  ble_bluenrg_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t ble_bluenrg_poll(void)
{
  /* Spaeter: BLE-Ereignisse abarbeiten, ohne die Messschleife zu blockieren. */
  return APP_STATUS_NOT_READY;
}

app_status_t ble_bluenrg_send_sample(const app_sample_t *sample)
{
  /* Spaeter: den aktuellen Messdatensatz als BLE-Paket oder Stream senden. */
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool ble_bluenrg_is_ready(void)
{
  return ble_bluenrg_ready;
}
