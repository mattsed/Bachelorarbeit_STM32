#ifndef BRAKE_PRESSURE_H
#define BRAKE_PRESSURE_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t brake_pressure_init(void);
app_status_t brake_pressure_read(brake_pressure_data_t *data);
bool brake_pressure_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BRAKE_PRESSURE_H */