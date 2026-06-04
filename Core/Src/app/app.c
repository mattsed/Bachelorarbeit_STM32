#include "app/app.h"

#include "board/board.h"
#include "comms/ble_bluenrg.h"
#include "sensors/acc_adxl373.h"
#include "sensors/brake_pressure.h"
#include "sensors/gnss.h"
#include "sensors/imu_lsm6dso.h"
#include "storage/storage_logger.h"

app_status_t app_init(void)
{
  (void)board_init();
  (void)gnss_init();
  (void)brake_pressure_init();
  (void)imu_lsm6dso_init();
  (void)acc_adxl373_init();
  (void)storage_logger_init();
  (void)ble_bluenrg_init();

  return APP_STATUS_OK;
}

void app_run(void)
{
  (void)gnss_poll();
  (void)ble_bluenrg_poll();
}