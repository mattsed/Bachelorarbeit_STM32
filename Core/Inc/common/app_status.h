#ifndef APP_STATUS_H
#define APP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  APP_STATUS_OK = 0,
  APP_STATUS_ERROR,
  APP_STATUS_NOT_READY,
  APP_STATUS_TIMEOUT
} app_status_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_STATUS_H */