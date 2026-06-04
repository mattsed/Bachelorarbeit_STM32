#ifndef BLE_BLUENRG_H
#define BLE_BLUENRG_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t ble_bluenrg_init(void);
app_status_t ble_bluenrg_poll(void);
app_status_t ble_bluenrg_send_sample(const app_sample_t *sample);
bool ble_bluenrg_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_BLUENRG_H */