#include "storage/storage_logger.h"

/* Wird true, sobald microSD/FatFS initialisiert ist und geschrieben werden kann. */
static bool storage_logger_ready = false;

app_status_t storage_logger_init(void)
{
  /* Spaeter: microSD ueber SPI5 initialisieren, FatFS mounten und Logdatei vorbereiten. */
  storage_logger_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t storage_logger_write_sample(const app_sample_t *sample)
{
  /* Spaeter: app_sample_t als CSV- oder Binaerdatensatz auf die microSD schreiben. */
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool storage_logger_is_ready(void)
{
  return storage_logger_ready;
}
