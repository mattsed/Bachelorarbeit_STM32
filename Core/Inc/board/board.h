#ifndef BOARD_H
#define BOARD_H

#include "common/app_status.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef hlpuart1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi5;

typedef struct
{
  UART_HandleTypeDef *gnss_uart;
  ADC_HandleTypeDef *brake_adc;
  SPI_HandleTypeDef *ble_spi;
  SPI_HandleTypeDef *microsd_spi;
  SPI_HandleTypeDef *reserved_sensor_spi;
} board_interfaces_t;

const board_interfaces_t *board_get_interfaces(void);
app_status_t board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */