#include "board/board.h"

static const board_interfaces_t board_interfaces =
{
  .gnss_uart = &hlpuart1,
  .brake_adc = &hadc1,
  .ble_spi = &hspi1,
  .microsd_spi = &hspi3,
  .reserved_sensor_spi = &hspi5
};

const board_interfaces_t *board_get_interfaces(void)
{
  return &board_interfaces;
}

app_status_t board_init(void)
{
  /* CubeMX initializes the low-level peripherals before this module is called. */
  return APP_STATUS_OK;
}