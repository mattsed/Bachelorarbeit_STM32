#ifndef STORAGE_LOGGER_H
#define STORAGE_LOGGER_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t storage_logger_init(void);
app_status_t storage_logger_write_sample(const app_sample_t *sample);
bool storage_logger_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_LOGGER_H */