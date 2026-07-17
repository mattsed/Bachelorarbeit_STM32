#include "app/app.h"

#include <stdio.h>
#include <string.h>

#include "board/board.h"
#include "comms/ble_bluenrg.h"
#include "sensors/acc_adxl373.h"
#include "sensors/brake_pressure.h"
#include "sensors/gnss.h"
#include "sensors/imu_lsm6dso.h"
#include "storage/storage_logger.h"

/* Messtakt: alle 20 ms ein kompletter Datensatz (50 Hz). GNSS aktualisiert
 * sich selbst nur mit 1 Hz; jede Zeile traegt den letzten bekannten Stand. */
#define APP_SAMPLE_PERIOD_MS  20u
#define APP_LED_PERIOD_MS     500u
#define APP_BUTTON_DEBOUNCE_MS 50u

/* B1-USER-Knopf (PC13): stoppt bzw. startet die Aufzeichnung. Ausgewertet
 * wird der entprellte Wechsel auf "gedrueckt" (Pegel high). */
static void app_check_user_button(void)
{
  static GPIO_PinState stable_state = GPIO_PIN_RESET;
  static GPIO_PinState last_raw = GPIO_PIN_RESET;
  static uint32_t last_change_ms = 0;

  GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  uint32_t now = HAL_GetTick();

  if (raw != last_raw)
  {
    last_raw = raw;
    last_change_ms = now;
    return;
  }
  if ((now - last_change_ms) < APP_BUTTON_DEBOUNCE_MS || raw == stable_state)
  {
    return;
  }
  stable_state = raw;

  if (stable_state == GPIO_PIN_SET) /* Knopf gedrueckt */
  {
    if (storage_logger_is_ready())
    {
      (void)storage_logger_stop();
    }
    else
    {
      (void)storage_logger_start();
    }
  }
}

app_status_t app_init(void)
{
  (void)board_init();
  (void)gnss_init();
  (void)brake_pressure_init();
  (void)imu_lsm6dso_init();
  (void)acc_adxl373_init();
  (void)storage_logger_init();
  (void)ble_bluenrg_init();

  if (storage_logger_is_ready())
  {
    printf("[App] Messzyklus laeuft (%u Hz).\r\n", (unsigned int)(1000u / APP_SAMPLE_PERIOD_MS));
  }
  else
  {
    printf("[App] Kein Logging moeglich (microSD nicht bereit) -- nur Sensor-Poll.\r\n");
  }

  return APP_STATUS_OK;
}

void app_run(void)
{
  static uint32_t next_sample_ms = 0;
  static uint32_t next_led_ms = 0;

  /* Hintergrundaufgaben: NMEA-Strom leeren, BLE-Ereignisse abholen,
   * Start/Stopp-Knopf auswerten. */
  (void)gnss_poll();
  (void)ble_bluenrg_poll();
  app_check_user_button();

  uint32_t now = HAL_GetTick();

  /* Gruene LED (PB0): blinkt = Aufzeichnung laeuft, dauerhaft an = gestoppt. */
  if (storage_logger_is_ready())
  {
    if (now >= next_led_ms)
    {
      next_led_ms = now + APP_LED_PERIOD_MS;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  }

  if (now < next_sample_ms)
  {
    return;
  }
  next_sample_ms = (next_sample_ms == 0u) ? now + APP_SAMPLE_PERIOD_MS
                                          : next_sample_ms + APP_SAMPLE_PERIOD_MS;

  app_sample_t sample;
  memset(&sample, 0, sizeof(sample));
  sample.timestamp_ms = now;
  (void)gnss_read(&sample.gnss);
  (void)brake_pressure_read(&sample.brake_pressure);
  (void)imu_lsm6dso_read(&sample.imu);
  (void)acc_adxl373_read(&sample.acc_400g);

  (void)storage_logger_write_sample(&sample);
}
