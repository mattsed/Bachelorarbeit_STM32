
#ifndef BOARD_H
#define BOARD_H

#include <stddef.h>
#include "common/app_status.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef hlpuart1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi5;

/* Beschreibt einen einzelnen GPIO-Pin zusammen mit seinem Port. */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} board_gpio_t;

/* Zentrale Zuordnung der von CubeMX erzeugten HAL-Handles zur Anwendung.
 * Hier steht, welche Schnittstelle fuer welches externe Modul vorgesehen ist:
 * - LPUART1: GNSS
 * - ADC1: Bremsdruck
 * - SPI3: IMU und ADXL373
 * - SPI5: microSD
 *
 * SPI1 war fuer das BLE-Modul vorgesehen; BLE wurde am 12.08.2026 aus dem
 * System genommen (Datenweg ist die microSD), der Treiber ist entfernt.
 * Die CubeMX-Konfiguration von SPI1 und den BLE-Pins besteht noch, wird
 * aber von der Anwendung nicht mehr angefasst.
 */
typedef struct
{
  UART_HandleTypeDef *gnss_uart;
  ADC_HandleTypeDef *brake_adc;
  SPI_HandleTypeDef *microsd_spi;
  SPI_HandleTypeDef *sensor_spi;
  board_gpio_t microsd_cs;
  board_gpio_t acc_cs;
  board_gpio_t imu_cs;
} board_interfaces_t;

const board_interfaces_t *board_get_interfaces(void);
app_status_t board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
