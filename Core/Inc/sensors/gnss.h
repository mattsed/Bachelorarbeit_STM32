#ifndef GNSS_H
#define GNSS_H

#include <stdbool.h>
#include "common/app_data.h"
#include "common/app_status.h"

#ifdef __cplusplus
extern "C" {
#endif

app_status_t gnss_init(void);
app_status_t gnss_poll(void);
app_status_t gnss_read(gnss_data_t *data);
bool gnss_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_H */