#include "storage/storage_logger.h"

static bool storage_logger_ready = false;

app_status_t storage_logger_init(void)
{
  storage_logger_ready = false;
  return APP_STATUS_NOT_READY;
}

app_status_t storage_logger_write_sample(const app_sample_t *sample)
{
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool storage_logger_is_ready(void)
{
  return storage_logger_ready;
}