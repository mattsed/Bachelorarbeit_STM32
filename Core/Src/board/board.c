#include "board/board.h"

/* Feste Board-Zuordnung fuer diese Hardwarekonfiguration.
 * Die HAL-Handles selbst werden weiterhin von CubeMX in main.c erzeugt.
 */
static const board_interfaces_t board_interfaces =
{
  .gnss_uart = &hlpuart1,
  .brake_adc = &hadc1,
  .ble_spi = &hspi1,
  .microsd_spi = &hspi5,
  .sensor_spi = &hspi3,
  .ble_cs = { .port = BLE_CS_GPIO_Port, .pin = BLE_CS_Pin },
  .ble_irq = { .port = BLE_IRQ_GPIO_Port, .pin = BLE_IRQ_Pin },
  .ble_reset = { .port = BLE_RESET_GPIO_Port, .pin = BLE_RESET_Pin },
  .microsd_cs = { .port = microSD_CS_GPIO_Port, .pin = microSD_CS_Pin },
  .acc_cs = { .port = ACC_CS_GPIO_Port, .pin = ACC_CS_Pin },
  .imu_cs = { .port = IMU_CS_GPIO_Port, .pin = IMU_CS_Pin }
};

const board_interfaces_t *board_get_interfaces(void)
{
  /* Module koennen ueber diesen Pointer auf die zentralen Interfaces zugreifen. */
  return &board_interfaces;
}

app_status_t board_init(void)
{
  /* CubeMX initialisiert die Low-Level-Peripherie bereits vor app_init(). */
  return APP_STATUS_OK;
}
