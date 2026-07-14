#ifndef BLE_BLUENRG_H
#define BLE_BLUENRG_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialisiert das BlueNRG-BLE-Modul ueber SPI1, IRQ und Reset. */
app_status_t ble_bluenrg_init(void);

/* Bearbeitet BLE-Ereignisse im Hintergrund, ohne den Messzyklus zu blockieren. */
app_status_t ble_bluenrg_poll(void);

/* Sendet einen kompletten Messdatensatz ueber BLE, falls eine Verbindung besteht. */
app_status_t ble_bluenrg_send_sample(const app_sample_t *sample);

/* Meldet, ob BLE initialisiert und sendebereit ist. */
bool ble_bluenrg_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_BLUENRG_H */
