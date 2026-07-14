#ifndef APP_H
#define APP_H

#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialisiert die projektbezogenen Module nach der CubeMX-Hardwareinitialisierung. */
app_status_t app_init(void);

/* Wird zyklisch aus der main-Schleife aufgerufen und steuert spaeter den Messablauf. */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
