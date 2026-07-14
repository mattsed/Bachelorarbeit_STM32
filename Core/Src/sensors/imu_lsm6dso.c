#include "sensors/imu_lsm6dso.h"

#include <stdio.h>

#include "board/board.h"

/* LSM6DSO-Register und SPI-Read-Bit (siehe LSM6DSO-Datenblatt, Register 0x0F). */
#define LSM6DSO_REG_WHO_AM_I  0x0Fu
#define LSM6DSO_WHO_AM_I_VAL  0x6Cu
#define LSM6DSO_SPI_READ_BIT  0x80u

/* Wird true, sobald die LSM6DSO-IMU ueber SPI3 erkannt und konfiguriert ist. */
static bool imu_lsm6dso_ready = false;

static void imu_lsm6dso_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->imu_cs.port, board->imu_cs.pin, state);
}

/* Liest ein einzelnes LSM6DSO-Register per SPI (Adressbyte mit gesetztem Read-Bit). */
static app_status_t imu_lsm6dso_read_reg(const board_interfaces_t *board, uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] = { (uint8_t)(reg | LSM6DSO_SPI_READ_BIT), 0x00u };
  uint8_t rx[2] = { 0 };

  imu_lsm6dso_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(board->sensor_spi, tx, rx, sizeof(tx), 10);
  imu_lsm6dso_cs(board, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  *value = rx[1];
  return APP_STATUS_OK;
}

app_status_t imu_lsm6dso_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  uint8_t who_am_i = 0;

  imu_lsm6dso_ready = false;

  if (imu_lsm6dso_read_reg(board, LSM6DSO_REG_WHO_AM_I, &who_am_i) != APP_STATUS_OK)
  {
    printf("[LSM6DSO] SPI-Fehler beim Lesen von WHO_AM_I.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[LSM6DSO] WHO_AM_I=0x%02X (erwartet 0x%02X)\r\n", who_am_i, LSM6DSO_WHO_AM_I_VAL);

  if (who_am_i != LSM6DSO_WHO_AM_I_VAL)
  {
    printf("[LSM6DSO] nicht erkannt.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[LSM6DSO] erkannt.\r\n");
  imu_lsm6dso_ready = true;
  return APP_STATUS_OK;
}

app_status_t imu_lsm6dso_read(imu_data_t *data)
{
  /* Spaeter: Beschleunigungs- und Gyro-Rohwerte aus dem LSM6DSO lesen. */
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool imu_lsm6dso_is_ready(void)
{
  return imu_lsm6dso_ready;
}
