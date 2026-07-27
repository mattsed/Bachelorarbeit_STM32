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

/* Gyro-Bias-Kalibrierung beim Start: Jedes Gyroskop hat einen kleinen
 * Nullpunktfehler (zeigt Drehung an, obwohl nichts dreht). Beim Einschalten
 * steht das Rad still -- also den Mittelwert ueber ~2 s messen und als Bias
 * merken. Die CSV bekommt weiterhin ROHwerte (Messdaten bleiben unveraendert,
 * sauber fuer die Thesis); der Bias wandert als Kommentarzeile in den
 * Dateikopf und wird erst in der PC-Auswertung abgezogen. */
#define LSM6DSO_BIAS_SAMPLES   200u
#define LSM6DSO_BIAS_DISCARD   10u   /* Einschwingzeit der Sensorfilter */
#define LSM6DSO_BIAS_GAP_MS    10u   /* etwas ueber der 104-Hz-Datenrate */

/* Wird true, sobald die LSM6DSO-IMU ueber SPI3 erkannt und konfiguriert ist. */
static bool imu_lsm6dso_ready = false;

static bool imu_bias_valid = false;
static int16_t imu_gyro_bias[3] = { 0, 0, 0 };


static void imu_lsm6dso_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->imu_cs.port, board->imu_cs.pin, state);
}


/* Liest ein einzelnes LSM6DSO-Register per SPI (Adressbyte mit gesetztem Read-Bit). */
static app_status_t imu_lsm6dso_read_reg(const board_interfaces_t *board, uint8_t reg, uint8_t *value)
{
  /* Dummy-Byte 0xA5 statt 0x00: erlaubt einen MISO-MOSI-Loopback-Test
   * (der Sensor ignoriert das Byte waehrend der Lesephase ohnehin). */
  uint8_t tx[2] = { (uint8_t)(reg | LSM6DSO_SPI_READ_BIT), 0xA5u }; /*fachnummer + leseanfrage als kleine zahl zw 0...255 (vorgabe durch sensor), dummy-bit*/
  uint8_t rx[2] = { 0 };

  /* SPI ist vollduplex: Waehrend Byte 1 (Adresse) gesendet wird, kommt
   * rx[0] herein (bedeutungslos); waehrend des Fuellbytes antwortet der
   * Sensor mit dem Registerinhalt in rx[1]. CS umrahmt die Transaktion. */
  imu_lsm6dso_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(board->sensor_spi, tx, rx, sizeof(tx), 10); /* Transaktion! (wenn nach 10ms nichts passiert, abbruch). sizeof falls sich arraygröße ändert */
  imu_lsm6dso_cs(board, GPIO_PIN_SET);

  if (result != HAL_OK) /*result inhalt: geklappt oder nicht?*/
  {
    return APP_STATUS_ERROR;
  }

  *value = rx[1]; /* antwort aus dem zweiten fach wird genommen (erstes fach enthält müll da sensor im austausch die frage noch nicht verstanden hatte) */
  return APP_STATUS_OK;
}


/* Schreibt ein einzelnes LSM6DSO-Register per SPI. */
static app_status_t imu_lsm6dso_write_reg(const board_interfaces_t *board, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { reg, value }; /* das register reg soll mit value beschrieben werden*/

  imu_lsm6dso_cs(board, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(board->sensor_spi, tx, sizeof(tx), 10);
  imu_lsm6dso_cs(board, GPIO_PIN_SET);

  return (result == HAL_OK) ? APP_STATUS_OK : APP_STATUS_ERROR; /*wenn result gleich HAL_OK ausgabe: APP_STATUS_OK, sonst APP_STATUS_ERROR*/
}


app_status_t imu_lsm6dso_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  uint8_t who_am_i = 0;

  imu_lsm6dso_ready = false;

  if (imu_lsm6dso_read_reg(board, LSM6DSO_REG_WHO_AM_I, &who_am_i) != APP_STATUS_OK) /* wer bist du?*/
  {
    printf("[LSM6DSO] SPI-Fehler beim Lesen von WHO_AM_I.\r\n");
    return APP_STATUS_ERROR;
  }

  if (who_am_i != LSM6DSO_WHO_AM_I_VAL) /* bist du der richtige? */
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

  /* Gyro-Bias im Stand messen (~2 s): erste Samples verwerfen (Filter
   * schwingen ein), dann Mittelwert ueber alle drei Achsen bilden. */
  int32_t sum[3] = { 0, 0, 0 };
  uint32_t used = 0;
  for (uint32_t i = 0; i < LSM6DSO_BIAS_SAMPLES; ++i)
  {
    imu_data_t d;
    HAL_Delay(LSM6DSO_BIAS_GAP_MS);
    if (imu_lsm6dso_read(&d) != APP_STATUS_OK)
    {
      continue;
    }
    if (i < LSM6DSO_BIAS_DISCARD)
    {
      continue;
    }
    sum[0] += d.gyro_x_raw;
    sum[1] += d.gyro_y_raw;
    sum[2] += d.gyro_z_raw;
    ++used;
  }
  if (used > (LSM6DSO_BIAS_SAMPLES / 2u))
  {
    imu_gyro_bias[0] = (int16_t)(sum[0] / (int32_t)used);
    imu_gyro_bias[1] = (int16_t)(sum[1] / (int32_t)used);
    imu_gyro_bias[2] = (int16_t)(sum[2] / (int32_t)used);
    imu_bias_valid = true;
    /* 1 LSB = 70 mdps -> Ausgabe direkt in Milligrad/s. */
    printf("[LSM6DSO] Gyro-Bias (Stand, %lu Samples): %ld / %ld / %ld mdps\r\n",
           (unsigned long)used,
           (long)imu_gyro_bias[0] * 70, (long)imu_gyro_bias[1] * 70,
           (long)imu_gyro_bias[2] * 70);
  }
  else
  {
    printf("[LSM6DSO] Gyro-Bias-Kalibrierung uebersprungen (zu wenige Samples).\r\n");
  }

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

bool imu_lsm6dso_get_gyro_bias(int16_t *gx, int16_t *gy, int16_t *gz)
{
  if (!imu_bias_valid || gx == NULL || gy == NULL || gz == NULL)
  {
    return false;
  }
  *gx = imu_gyro_bias[0];
  *gy = imu_gyro_bias[1];
  *gz = imu_gyro_bias[2];
  return true;
}
