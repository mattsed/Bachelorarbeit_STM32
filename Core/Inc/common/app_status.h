#ifndef APP_STATUS_H
#define APP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Gemeinsame Rueckmeldung fuer alle Projektmodule.
 * Dadurch kann die app-Schicht einheitlich bewerten, ob eine Aktion
 * erfolgreich war, noch nicht bereit ist, fehlgeschlagen ist oder zu lange
 * gedauert hat.
 */
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
