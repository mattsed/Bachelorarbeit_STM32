#include "sensors/brake_pressure.h"

/* Wird true, sobald die ADC-Kanaele fuer die Bremsdrucksensoren bereit sind. */
static bool brake_pressure_ready = false;

app_status_t brake_pressure_init(void)
{
  /* Spaeter: ADC1 fuer vorderen/hinteren Bremsdruck vorbereiten. */
  brake_pressure_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t brake_pressure_read(brake_pressure_data_t *data)
{
  /* Spaeter: ADC-Rohwerte lesen und nach Kalibrierung in bar umrechnen. */
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool brake_pressure_is_ready(void)
{
  return brake_pressure_ready;
}
