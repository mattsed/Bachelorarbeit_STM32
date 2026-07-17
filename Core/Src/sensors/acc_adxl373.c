#include "sensors/acc_adxl373.h"

#include <stdio.h>

#include "board/board.h"

/* ADXL373-Register und SPI-Adressformat (Adresse<<1 | Read-Bit), siehe
 * ADXL372/373-Familie (Registeradressen mit ADIs no-OS-Treiber abgeglichen). */
#define ADXL373_REG_DEVID_AD    0x00u
#define ADXL373_REG_DEVID_MST   0x01u
#define ADXL373_REG_PARTID      0x02u
#define ADXL373_REG_X_DATA_H    0x08u  /* ab hier: X/Y/Z je H+L, 12 Bit linksbuendig */
#define ADXL373_REG_TIMING      0x3Du
#define ADXL373_REG_MEASURE     0x3Eu
#define ADXL373_REG_POWER_CTL   0x3Fu

#define ADXL373_DEVID_AD_VAL    0xADu
#define ADXL373_DEVID_MST_VAL   0x1Du

#define ADXL373_TIMING_VAL      0x00u  /* ODR 400 Hz */
#define ADXL373_MEASURE_VAL     0x00u  /* Bandbreite 200 Hz */
#define ADXL373_POWER_CTL_VAL   0x03u  /* Full-Bandwidth-Messbetrieb */

#define ADXL373_SPI_READ(reg)   (uint8_t)(((reg) << 1) | 0x01u)
#define ADXL373_SPI_WRITE(reg)  (uint8_t)((reg) << 1)

/* Wird true, sobald der ADXL373 ueber SPI3 erkannt und konfiguriert ist. */
static bool acc_adxl373_ready = false;

static void acc_adxl373_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->acc_cs.port, board->acc_cs.pin, state);
}

/* Liest ein einzelnes ADXL373-Register per SPI. */
static app_status_t acc_adxl373_read_reg(const board_interfaces_t *board, uint8_t reg, uint8_t *value)
{
  /* Dummy-Byte 0xA5 statt 0x00: erlaubt einen MISO-MOSI-Loopback-Test
   * (der Sensor ignoriert das Byte waehrend der Lesephase ohnehin). */
  uint8_t tx[2] = { ADXL373_SPI_READ(reg), 0xA5u };
  uint8_t rx[2] = { 0 };

  acc_adxl373_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(board->sensor_spi, tx, rx, sizeof(tx), 10);
  acc_adxl373_cs(board, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  *value = rx[1];
  return APP_STATUS_OK;
}

/* Schreibt ein einzelnes ADXL373-Register per SPI. */
static app_status_t acc_adxl373_write_reg(const board_interfaces_t *board, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { ADXL373_SPI_WRITE(reg), value };

  acc_adxl373_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(board->sensor_spi, tx, sizeof(tx), 10);
  acc_adxl373_cs(board, GPIO_PIN_SET);

  return (result == HAL_OK) ? APP_STATUS_OK : APP_STATUS_ERROR;
}

app_status_t acc_adxl373_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  uint8_t devid_ad = 0;
  uint8_t devid_mst = 0;
  uint8_t partid = 0;

  acc_adxl373_ready = false;

  if (acc_adxl373_read_reg(board, ADXL373_REG_DEVID_AD, &devid_ad) != APP_STATUS_OK ||
      acc_adxl373_read_reg(board, ADXL373_REG_DEVID_MST, &devid_mst) != APP_STATUS_OK ||
      acc_adxl373_read_reg(board, ADXL373_REG_PARTID, &partid) != APP_STATUS_OK)
  {
    printf("[ADXL373] SPI-Fehler beim Lesen der ID-Register.\r\n");
    return APP_STATUS_ERROR;
  }

  if (devid_ad != ADXL373_DEVID_AD_VAL || devid_mst != ADXL373_DEVID_MST_VAL)
  {
    printf("[ADXL373] DEVID_AD=0x%02X DEVID_MST=0x%02X PARTID=0x%02X -- nicht erkannt.\r\n",
           devid_ad, devid_mst, partid);
    return APP_STATUS_ERROR;
  }

  /* Messbetrieb konfigurieren: ODR 400 Hz, Bandbreite 200 Hz, Full-BW-Modus. */
  if (acc_adxl373_write_reg(board, ADXL373_REG_TIMING, ADXL373_TIMING_VAL) != APP_STATUS_OK ||
      acc_adxl373_write_reg(board, ADXL373_REG_MEASURE, ADXL373_MEASURE_VAL) != APP_STATUS_OK ||
      acc_adxl373_write_reg(board, ADXL373_REG_POWER_CTL, ADXL373_POWER_CTL_VAL) != APP_STATUS_OK)
  {
    printf("[ADXL373] Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[ADXL373] erkannt und konfiguriert (400 Hz, +/-400 g).\r\n");
  acc_adxl373_ready = true;
  return APP_STATUS_OK;
}

app_status_t acc_adxl373_read(acc_400g_data_t *data)
{
  const board_interfaces_t *board = board_get_interfaces();
  /* 1 Adressbyte + 6 Datenbytes (X/Y/Z je H+L, Auto-Inkrement). */
  uint8_t tx[7] = { ADXL373_SPI_READ(ADXL373_REG_X_DATA_H) };
  uint8_t rx[7] = { 0 };

  if (!acc_adxl373_ready || data == NULL)
  {
    return APP_STATUS_NOT_READY;
  }

  acc_adxl373_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(board->sensor_spi, tx, rx, sizeof(tx), 10);
  acc_adxl373_cs(board, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  /* 12-Bit-Werte, linksbuendig in 16 Bit -> arithmetisch um 4 schieben
   * (erhaelt das Vorzeichen). 1 LSB = 200 mg. */
  data->accel_x_raw = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]) >> 4;
  data->accel_y_raw = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]) >> 4;
  data->accel_z_raw = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]) >> 4;
  return APP_STATUS_OK;
}

bool acc_adxl373_is_ready(void)
{
  return acc_adxl373_ready;
}
