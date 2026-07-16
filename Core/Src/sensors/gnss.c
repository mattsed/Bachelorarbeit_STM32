#include "sensors/gnss.h"

#include <stdio.h>

#include "board/board.h"

/* Der Teseo-LIV4F auf dem X-NUCLEO-LIV4A1 ist auf diesem Board ueber I2C
 * angebunden (Arduino D15/D14 = PB8/PB9, 7-Bit-Adresse 0x3A) -- so macht es
 * auch STs eigenes Beispielprojekt fuer das NUCLEO-H563ZI. Der NMEA-Strom
 * kann direkt per I2C-Read gelesen werden; solange keine Daten anstehen,
 * liefert das Modul 0xFF-Fuellbytes.
 *
 * Fuer das Bring-up genuegt Software-I2C auf PB8/PB9 (Open-Drain, interne
 * Pull-ups); spaeter kann das auf das I2C1-Peripheral umgestellt werden. */
#define GNSS_I2C_ADDR           0x3Au
#define GNSS_SCL_PIN            GPIO_PIN_8
#define GNSS_SDA_PIN            GPIO_PIN_9
#define GNSS_COLLECT_TIMEOUT_MS 3000u
#define GNSS_BUF_LEN            512u

/* Wird true, sobald der GNSS-Treiber initialisiert ist und verwertbare Daten
 * liefern kann. */
static bool gnss_ready = false;

static void gnss_i2c_delay(void)
{
  for (volatile uint32_t i = 0; i < 400u; ++i)
  {
    __NOP();
  }
}

static void gnss_i2c_scl(int high)
{
  HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void gnss_i2c_sda(int high)
{
  HAL_GPIO_WritePin(GPIOB, GNSS_SDA_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int gnss_i2c_sda_read(void)
{
  return (HAL_GPIO_ReadPin(GPIOB, GNSS_SDA_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

static void gnss_i2c_start(void)
{
  gnss_i2c_sda(1);
  gnss_i2c_scl(1);
  gnss_i2c_delay();
  gnss_i2c_sda(0);
  gnss_i2c_delay();
  gnss_i2c_scl(0);
  gnss_i2c_delay();
}

static void gnss_i2c_stop(void)
{
  gnss_i2c_sda(0);
  gnss_i2c_delay();
  gnss_i2c_scl(1);
  gnss_i2c_delay();
  gnss_i2c_sda(1);
  gnss_i2c_delay();
}

/* Schreibt ein Byte; Rueckgabe true, wenn der Slave mit ACK antwortet. */
static bool gnss_i2c_write_byte(uint8_t byte)
{
  for (int i = 7; i >= 0; --i)
  {
    gnss_i2c_sda((byte >> i) & 1u);
    gnss_i2c_delay();
    gnss_i2c_scl(1);
    gnss_i2c_delay();
    gnss_i2c_scl(0);
    gnss_i2c_delay();
  }

  gnss_i2c_sda(1);
  gnss_i2c_delay();
  gnss_i2c_scl(1);
  gnss_i2c_delay();
  bool ack = (gnss_i2c_sda_read() == 0);
  gnss_i2c_scl(0);
  gnss_i2c_delay();
  return ack;
}

/* Liest ein Byte; ack=true sendet Master-ACK (weitere Bytes folgen). */
static uint8_t gnss_i2c_read_byte(bool ack)
{
  uint8_t byte = 0;

  gnss_i2c_sda(1); /* SDA freigeben */
  for (int i = 7; i >= 0; --i)
  {
    gnss_i2c_delay();
    gnss_i2c_scl(1);
    gnss_i2c_delay();
    if (gnss_i2c_sda_read())
    {
      byte |= (uint8_t)(1u << i);
    }
    gnss_i2c_scl(0);
  }

  gnss_i2c_sda(ack ? 0 : 1);
  gnss_i2c_delay();
  gnss_i2c_scl(1);
  gnss_i2c_delay();
  gnss_i2c_scl(0);
  gnss_i2c_sda(1);
  gnss_i2c_delay();
  return byte;
}

static void gnss_i2c_pins_init(void)
{
  GPIO_InitTypeDef gpio = { 0 };

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GNSS_SCL_PIN | GNSS_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  gnss_i2c_scl(1);
  gnss_i2c_sda(1);
  gnss_i2c_delay();
}

/* Liest einen Block NMEA-Bytes vom Teseo; 0xFF-Fuellbytes werden verworfen.
 * Rueckgabe: Anzahl uebernommener Nutzbytes. */
static uint32_t gnss_i2c_read_chunk(uint8_t *dst, uint32_t max)
{
  uint32_t n = 0;

  gnss_i2c_start();
  if (!gnss_i2c_write_byte((uint8_t)((GNSS_I2C_ADDR << 1) | 1u)))
  {
    gnss_i2c_stop();
    return 0;
  }
  for (uint32_t i = 0; i < 32u; ++i)
  {
    uint8_t b = gnss_i2c_read_byte(i < 31u);
    if (b != 0xFFu && n < max)
    {
      dst[n++] = b;
    }
  }
  gnss_i2c_stop();
  return n;
}

/* Gibt die erste vollstaendige NMEA-Zeile aus dem Puffer aus. */
static void gnss_print_first_sentence(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    if (buf[i] != '$')
    {
      continue;
    }
    printf("[GNSS] Beispielsatz: ");
    for (uint32_t j = i; j < len && buf[j] != '\r' && buf[j] != '\n' && (j - i) < 82u; ++j)
    {
      putchar(buf[j]);
    }
    printf("\r\n");
    return;
  }
}

app_status_t gnss_init(void)
{
  static uint8_t buf[GNSS_BUF_LEN];
  uint32_t len = 0;
  bool has_nmea = false;

  gnss_ready = false;

  gnss_i2c_pins_init();

  /* Erreichbarkeit pruefen (Adress-ACK). */
  gnss_i2c_start();
  bool present = gnss_i2c_write_byte((uint8_t)(GNSS_I2C_ADDR << 1));
  gnss_i2c_stop();
  if (!present)
  {
    printf("[GNSS] Teseo antwortet nicht auf I2C-Adresse 0x%02X.\r\n", GNSS_I2C_ADDR);
    return APP_STATUS_ERROR;
  }

  /* NMEA-Strom einsammeln (das Modul sendet im Sekundentakt). */
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < GNSS_COLLECT_TIMEOUT_MS && len < GNSS_BUF_LEN)
  {
    len += gnss_i2c_read_chunk(&buf[len], GNSS_BUF_LEN - len);
    for (uint32_t i = 1; i < len; ++i)
    {
      if (buf[i - 1] == '$' && buf[i] == 'G')
      {
        has_nmea = true;
      }
    }
    if (has_nmea && len > 80u)
    {
      break;
    }
  }

  if (!has_nmea)
  {
    printf("[GNSS] Teseo auf I2C erreichbar, aber kein NMEA empfangen (%lu Bytes).\r\n",
           (unsigned long)len);
    return APP_STATUS_ERROR;
  }

  printf("[GNSS] Teseo-LIV4F ueber I2C erkannt, NMEA-Strom aktiv (%lu Bytes).\r\n",
         (unsigned long)len);
  gnss_print_first_sentence(buf, len);
  gnss_ready = true;
  return APP_STATUS_OK;
}

app_status_t gnss_poll(void)
{
  /* Spaeter: empfangene NMEA-Zeichen verarbeiten, ohne die Hauptschleife zu blockieren. */
  return gnss_ready ? APP_STATUS_OK : APP_STATUS_NOT_READY;
}

app_status_t gnss_read(gnss_data_t *data)
{
  /* Spaeter: zuletzt geparste Position, Geschwindigkeit und UTC-Zeit ausgeben. */
  (void)data;
  return APP_STATUS_NOT_READY;
}

bool gnss_is_ready(void)
{
  return gnss_ready;
}
