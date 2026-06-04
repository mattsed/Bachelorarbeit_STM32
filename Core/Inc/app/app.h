#ifndef APP_H
#define APP_H

#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t app_init(void);
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */