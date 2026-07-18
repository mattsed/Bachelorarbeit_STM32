#include "sensors/imu_lsm6dso.h"

#include <stdio.h>

#include "board/board.h"

/* LSM6DSO-Register (siehe Datenblatt).
 * Konfiguration fuer den Downhill-Einsatz: 104 Hz Abtastrate,
 * Beschleunigung +/-16 g, Drehrate +/-2000 dps. */
#define LSM6DSO_REG_WHO_AM_I  0x0Fu
#define LSM6DSO_REG_CTRL1_XL  0x10u
#define LSM6DSO_REG_CTRL2_G   0x11u
#define LSM6DSO_REG_CTRL3_C   0x12u
#define LSM6DSO_REG_OUTX_L_G  0x22u  /* ab hier: Gyro XYZ, dann Accel XYZ */

#define LSM6DSO_WHO_AM_I_VAL  0x6Cu
#define LSM6DSO_SPI_READ_BIT  0x80u

#define LSM6DSO_CTRL1_XL_VAL  0x44u  /* ODR 104 Hz, FS +/-16 g */
#define LSM6DSO_CTRL2_G_VAL   0x4Cu  /* ODR 104 Hz, FS +/-2000 dps */
#define LSM6DSO_CTRL3_C_VAL   0x44u  /* BDU + Adress-Auto-Inkrement */

/* Wird true, sobald die LSM6DSO-IMU ueber SPI3 erkannt und konfiguriert ist. */
static bool imu_lsm6dso_ready = false;

static void imu_lsm6dso_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->imu_cs.port, board->imu_cs.pin, state);
}

/* Liest ein einzelnes LSM6DSO-Register per SPI (Adressbyte mit gesetztem Read-Bit). */
static app_status_t imu_lsm6dso_read_reg(const board_interfaces_t *board, uint8_t reg, uint8_t *value)
{
  /* Dummy-Byte 0xA5 statt 0x00: erlaubt einen MISO-MOSI-Loopback-Test
   * (der Sensor ignoriert das Byte waehrend der Lesephase ohnehin). */
  uint8_t tx[2] = { (uint8_t)(reg | LSM6DSO_SPI_READ_BIT), 0xA5u };
  uint8_t rx[2] = { 0 };

  /* SPI ist vollduplex: Waehrend Byte 1 (Adresse) gesendet wird, kommt
   * rx[0] herein (bedeutungslos); waehrend des Fuellbytes antwortet der
   * Sensor mit dem Registerinhalt in rx[1]. CS umrahmt die Transaktion. */
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

/* Schreibt ein einzelnes LSM6DSO-Register per SPI. */
static app_status_t imu_lsm6dso_write_reg(const board_interfaces_t *board, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { reg, value };

  imu_lsm6dso_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(board->sensor_spi, tx, sizeof(tx), 10);
  imu_lsm6dso_cs(board, GPIO_PIN_SET);

  return (result == HAL_OK) ? APP_STATUS_OK : APP_STATUS_ERROR;
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

  if (who_am_i != LSM6DSO_WHO_AM_I_VAL)
  {
    printf("[LSM6DSO] WHO_AM_I=0x%02X (erwartet 0x%02X) -- nicht erkannt.\r\n",
           who_am_i, LSM6DSO_WHO_AM_I_VAL);
    return APP_STATUS_ERROR;
  }

  /* Messbetrieb konfigurieren -- der Sensor startet im Schlafmodus.
   * Reihenfolge: erst CTRL3_C (BDU + Auto-Inkrement als Grundverhalten),
   * dann Accel und Gyro einschalten (je 104 Hz + Messbereich). */
  if (imu_lsm6dso_write_reg(board, LSM6DSO_REG_CTRL3_C, LSM6DSO_CTRL3_C_VAL) != APP_STATUS_OK ||
      imu_lsm6dso_write_reg(board, LSM6DSO_REG_CTRL1_XL, LSM6DSO_CTRL1_XL_VAL) != APP_STATUS_OK ||
      imu_lsm6dso_write_reg(board, LSM6DSO_REG_CTRL2_G, LSM6DSO_CTRL2_G_VAL) != APP_STATUS_OK)
  {
    printf("[LSM6DSO] Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[LSM6DSO] erkannt und konfiguriert (104 Hz, +/-16 g, +/-2000 dps).\r\n");
  imu_lsm6dso_ready = true;
  return APP_STATUS_OK;
}

app_status_t imu_lsm6dso_read(imu_data_t *data)
{
  const board_interfaces_t *board = board_get_interfaces();
  /* 1 Adressbyte + 12 Datenbytes: OUTX_L_G .. OUTZ_H_A (Auto-Inkrement). */
  uint8_t tx[13] = { (uint8_t)(LSM6DSO_REG_OUTX_L_G | LSM6DSO_SPI_READ_BIT) };
  uint8_t rx[13] = { 0 };

  if (!imu_lsm6dso_ready || data == NULL)
  {
    return APP_STATUS_NOT_READY;
  }

  imu_lsm6dso_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(board->sensor_spi, tx, rx, sizeof(tx), 10);
  imu_lsm6dso_cs(board, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  /* Bytes zu 16-Bit-Werten zusammensetzen: Der Sensor liefert little-endian
   * (niederwertiges Byte zuerst), Reihenfolge laut Registerkarte erst
   * Gyro X/Y/Z, dann Accel X/Y/Z. rx[0] ist das Echo des Adressbytes. */
  data->gyro_x_raw = (int16_t)((uint16_t)rx[1] | ((uint16_t)rx[2] << 8));
  data->gyro_y_raw = (int16_t)((uint16_t)rx[3] | ((uint16_t)rx[4] << 8));
  data->gyro_z_raw = (int16_t)((uint16_t)rx[5] | ((uint16_t)rx[6] << 8));
  data->accel_x_raw = (int16_t)((uint16_t)rx[7] | ((uint16_t)rx[8] << 8));
  data->accel_y_raw = (int16_t)((uint16_t)rx[9] | ((uint16_t)rx[10] << 8));
  data->accel_z_raw = (int16_t)((uint16_t)rx[11] | ((uint16_t)rx[12] << 8));
  return APP_STATUS_OK;
}

bool imu_lsm6dso_is_ready(void)
{
  return imu_lsm6dso_ready;
}
