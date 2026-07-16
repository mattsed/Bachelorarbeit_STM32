#include "sensors/acc_adxl373.h"

#include <stdio.h>

#include "board/board.h"

/* ADXL373-Register und SPI-Adressformat (Adresse<<1 | Read-Bit), siehe ADXL372/373-Familie. */
#define ADXL373_REG_DEVID_AD    0x00u
#define ADXL373_REG_DEVID_MST   0x01u
#define ADXL373_REG_PARTID      0x02u
#define ADXL373_DEVID_AD_VAL    0xADu
#define ADXL373_DEVID_MST_VAL   0x1Du
#define ADXL373_SPI_READ(reg)   (uint8_t)(((reg) << 1) | 0x01u)

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

  /* PARTID wird nur zur Kontrolle ausgegeben, da der genaue Datenblattwert
   * fuer den ADXL373 (im Gegensatz zu DEVID_AD/DEVID_MST) hier nicht
   * verifiziert werden konnte -- bitte mit Manuals/400g.pdf abgleichen. */
  printf("[ADXL373] DEVID_AD=0x%02X DEVID_MST=0x%02X PARTID=0x%02X\r\n", devid_ad, devid_mst, partid);

  if (devid_ad != ADXL373_DEVID_AD_VAL || devid_mst != ADXL373_DEVID_MST_VAL)
  {
    printf("[ADXL373] nicht erkannt.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[ADXL373] erkannt.\r\n");
  acc_adxl373_ready = true;
  return APP_STATUS_OK;
}

app_status_t acc_adxl373_read(acc_400g_data_t *data)
{
  /* Spaeter: 400g-Beschleunigungsrohdaten aus dem ADXL373 lesen. */
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool acc_adxl373_is_ready(void)
{
  return acc_adxl373_ready;
}
