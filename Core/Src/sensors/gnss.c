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
/* Grosszuegig bemessen: nach einem Board-Reset bootet auch der Teseo neu
 * (Reset-Netz der Shields) und braucht 1..2 s, bis NMEA wieder laeuft. */
#define GNSS_COLLECT_TIMEOUT_MS 6000u
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
  while ((HAL_GetTick() - start) < GNSS_COLLECT_TIMEOUT_MS)
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
    /* Puffer voll, aber kein NMEA darin (z. B. Boot-Muell direkt nach einem
     * Modul-Reset): verwerfen und weiter sammeln. */
    if (len >= GNSS_BUF_LEN && !has_nmea)
    {
      len = 0;
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

/* ==========================================================================
 * NMEA-Parser: sammelt Zeichen aus dem I2C-Strom zu Zeilen und wertet
 * RMC-Saetze aus (Position, Geschwindigkeit, UTC-Zeit, Fix-Status).
 * ========================================================================== */

#define GNSS_LINE_LEN     100u
#define GNSS_POLL_GAP_MS  5u

static gnss_data_t gnss_last;      /* zuletzt geparster Stand */
static char gnss_line[GNSS_LINE_LEN];
static uint32_t gnss_line_len = 0;
static uint32_t gnss_last_poll_ms = 0;

/* Zuletzt empfangenes UTC-Datum (RMC-Feld 9); 0 = noch keins empfangen. */
static uint8_t gnss_date_day = 0;
static uint8_t gnss_date_month = 0;
static uint16_t gnss_date_year = 0;

/* Zerlegt "ddmm.mmmm" (bzw. dddmm.mmmm) nach Grad * 10^7. */
static int32_t gnss_coord_to_e7(const char *field, int deg_digits)
{
  int32_t deg = 0;
  double minutes = 0.0;
  double frac_scale = 0.1;
  int i = 0;

  for (; i < deg_digits && field[i] >= '0' && field[i] <= '9'; ++i)
  {
    deg = deg * 10 + (field[i] - '0');
  }
  for (; field[i] >= '0' && field[i] <= '9'; ++i)
  {
    minutes = minutes * 10.0 + (double)(field[i] - '0');
  }
  if (field[i] == '.')
  {
    for (++i; field[i] >= '0' && field[i] <= '9'; ++i)
    {
      minutes += (double)(field[i] - '0') * frac_scale;
      frac_scale *= 0.1;
    }
  }
  return (int32_t)((double)deg * 1e7 + (minutes / 60.0) * 1e7 + 0.5);
}

/* Wertet einen kompletten RMC-Satz aus ("$GNRMC,hhmmss.sss,A,lat,N,lon,E,knoten,..."). */
static void gnss_parse_rmc(const char *line)
{
  /* Felder anhand der Kommas lokalisieren (Feld 0 = Satzname). */
  const char *field[13] = { 0 };
  int nfields = 0;

  field[nfields++] = line;
  for (const char *p = line; *p != '\0' && nfields < 13; ++p)
  {
    if (*p == ',')
    {
      field[nfields++] = p + 1;
    }
  }
  if (nfields < 10)
  {
    return;
  }

  /* Feld 1: UTC hhmmss.sss -> Millisekunden seit Mitternacht. */
  const char *t = field[1];
  if (t[0] >= '0' && t[0] <= '9')
  {
    uint32_t hh = (uint32_t)(t[0] - '0') * 10u + (uint32_t)(t[1] - '0');
    uint32_t mm = (uint32_t)(t[2] - '0') * 10u + (uint32_t)(t[3] - '0');
    uint32_t ss = (uint32_t)(t[4] - '0') * 10u + (uint32_t)(t[5] - '0');
    uint32_t ms = 0;
    if (t[6] == '.')
    {
      for (int i = 7; i < 10 && t[i] >= '0' && t[i] <= '9'; ++i)
      {
        ms = ms * 10u + (uint32_t)(t[i] - '0');
      }
    }
    gnss_last.utc_time_ms = ((hh * 60u + mm) * 60u + ss) * 1000u + ms;
  }

  /* Feld 2: Status A = Fix gueltig, V = ungueltig. */
  gnss_last.fix_valid = (field[2][0] == 'A');

  if (gnss_last.fix_valid)
  {
    /* Felder 3-6: Breite/Laenge mit Himmelsrichtung. */
    int32_t lat = gnss_coord_to_e7(field[3], 2);
    int32_t lon = gnss_coord_to_e7(field[5], 3);
    gnss_last.latitude_e7 = (field[4][0] == 'S') ? -lat : lat;
    gnss_last.longitude_e7 = (field[6][0] == 'W') ? -lon : lon;

    /* Feld 7: Geschwindigkeit in Knoten -> mm/s (1 kn = 514,444 mm/s). */
    double knots = 0.0;
    double frac = 0.1;
    const char *v = field[7];
    int i = 0;
    for (; v[i] >= '0' && v[i] <= '9'; ++i)
    {
      knots = knots * 10.0 + (double)(v[i] - '0');
    }
    if (v[i] == '.')
    {
      for (++i; v[i] >= '0' && v[i] <= '9'; ++i)
      {
        knots += (double)(v[i] - '0') * frac;
        frac *= 0.1;
      }
    }
    gnss_last.speed_mm_s = (uint32_t)(knots * 514.444 + 0.5);

    /* Feld 9: Datum ddmmyy -- nur bei gueltigem Fix uebernehmen,
     * ohne Fix kann das Feld leer oder veraltet sein. */
    const char *d = field[9];
    if (d[0] >= '0' && d[0] <= '9' && d[1] >= '0' && d[1] <= '9' &&
        d[2] >= '0' && d[2] <= '9' && d[3] >= '0' && d[3] <= '9' &&
        d[4] >= '0' && d[4] <= '9' && d[5] >= '0' && d[5] <= '9')
    {
      gnss_date_day = (uint8_t)((d[0] - '0') * 10 + (d[1] - '0'));
      gnss_date_month = (uint8_t)((d[2] - '0') * 10 + (d[3] - '0'));
      gnss_date_year = (uint16_t)(2000 + (d[4] - '0') * 10 + (d[5] - '0'));
    }
  }
}

/* Verarbeitet ein einzelnes Zeichen aus dem NMEA-Strom. */
static void gnss_feed_char(char c)
{
  if (c == '$')
  {
    gnss_line_len = 0;
  }
  if (c == '\n' || c == '\r')
  {
    if (gnss_line_len > 6u && gnss_line_len < GNSS_LINE_LEN)
    {
      gnss_line[gnss_line_len] = '\0';
      /* RMC-Saetze aller Konstellationen ($GNRMC, $GPRMC, ...). */
      if (gnss_line[0] == '$' && gnss_line[3] == 'R' && gnss_line[4] == 'M' && gnss_line[5] == 'C')
      {
        gnss_parse_rmc(gnss_line);
      }
    }
    gnss_line_len = 0;
    return;
  }
  if (gnss_line_len < GNSS_LINE_LEN - 1u)
  {
    gnss_line[gnss_line_len++] = c;
  }
}

app_status_t gnss_poll(void)
{
  uint8_t chunk[32];
  uint32_t now = HAL_GetTick();

  if (!gnss_ready)
  {
    return APP_STATUS_NOT_READY;
  }

  /* Lesetakt begrenzen, damit der Software-I2C die Hauptschleife nicht
   * dominiert; der NMEA-Strom (ca. 0,5 kB/s) wird trotzdem locker geleert. */
  if ((now - gnss_last_poll_ms) < GNSS_POLL_GAP_MS)
  {
    return APP_STATUS_OK;
  }
  gnss_last_poll_ms = now;

  uint32_t n = gnss_i2c_read_chunk(chunk, sizeof(chunk));
  for (uint32_t i = 0; i < n; ++i)
  {
    gnss_feed_char((char)chunk[i]);
  }
  return APP_STATUS_OK;
}

app_status_t gnss_read(gnss_data_t *data)
{
  if (!gnss_ready || data == NULL)
  {
    return APP_STATUS_NOT_READY;
  }
  *data = gnss_last;
  return APP_STATUS_OK;
}

bool gnss_get_utc_datetime(uint16_t *year, uint8_t *month, uint8_t *day,
                           uint8_t *hour, uint8_t *minute, uint8_t *second)
{
  if (gnss_date_year == 0u)
  {
    return false; /* noch kein Datum vom GNSS empfangen */
  }

  uint32_t total_s = gnss_last.utc_time_ms / 1000u;
  *year = gnss_date_year;
  *month = gnss_date_month;
  *day = gnss_date_day;
  *hour = (uint8_t)(total_s / 3600u);
  *minute = (uint8_t)((total_s / 60u) % 60u);
  *second = (uint8_t)(total_s % 60u);
  return true;
}

bool gnss_is_ready(void)
{
  return gnss_ready;
}
