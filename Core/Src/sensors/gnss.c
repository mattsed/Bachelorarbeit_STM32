#include "sensors/gnss.h"

/* Wird true, sobald der GNSS-Treiber initialisiert ist und verwertbare Daten
 * liefern kann. Im Stub bleibt das Modul bewusst NOT_READY.
 */
static bool gnss_ready = false;

app_status_t gnss_init(void)
{
  /* Spaeter: LPUART1/UART-Empfang starten und NMEA-Puffer vorbereiten. */
  gnss_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t gnss_poll(void)
{
  /* Spaeter: empfangene NMEA-Zeichen verarbeiten, ohne die Hauptschleife zu blockieren. */
  return APP_STATUS_NOT_READY;
}

app_status_t gnss_read(gnss_data_t *data)
{
  /* Spaeter: zuletzt geparste Position, Geschwindigkeit und UTC-Zeit ausgeben. */
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool gnss_is_ready(void)
{
  return gnss_ready;
}
