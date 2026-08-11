#include "sensors/gnss.h"

#include <stdio.h>
#include <string.h>

#include "board/board.h"

/* Der Teseo-LIV4F auf dem X-NUCLEO-LIV4A1 ist auf diesem Board ueber I2C
 * angebunden (Arduino D15/D14 = PB8/PB9, 7-Bit-Adresse 0x3A) -- so macht es
 * auch STs eigenes Beispielprojekt fuer das NUCLEO-H563ZI. Der NMEA-Strom
 * kann direkt per I2C-Read gelesen werden; solange keine Daten anstehen,
 * liefert das Modul 0xFF-Fuellbytes.
 *
 * Seit dem Bring-up vom anfaenglichen Software-I2C (Bit-Banging: Pins per
 * Code von Hand takten) auf das Hardware-I2C1-Peripheral umgestellt:
 * dieselben Pins PB8/PB9 gehoeren als Alternativfunktion AF4 zu I2C1, das
 * das Protokoll selbststaendig abwickelt. Vorteil: ~380 kHz Bustakt statt
 * des langsamen Bit-Bangings -- pro 32-Byte-Block wartet die CPU nur noch
 * ~1 ms, damit erreicht die Messschleife die vollen 50 Hz.
 *
 * Direkte Registerzugriffe statt HAL-I2C, weil das I2C-HAL-Modul im
 * CubeMX-Projekt deaktiviert ist und eine Aktivierung in
 * stm32h5xx_hal_conf.h bei der naechsten Code-Generierung verloren ginge.
 * Kernel-Takt: CSI (4 MHz) -- unabhaengig von der APB-Konfiguration,
 * gleiche Strategie wie bei LPUART1 und der SD-Initialisierung. */
#define GNSS_I2C_ADDR           0x3Au
#define GNSS_SCL_PIN            GPIO_PIN_8
#define GNSS_SDA_PIN            GPIO_PIN_9

/* TIMINGR fuer 4-MHz-Kerneltakt: PRESC=0, SCLL=5 (1,5 us low), SCLH=2
 * (0,75 us high), SCLDEL=1, SDADEL=1 -> ca. 380 kHz (Fast-Mode). */
#define GNSS_I2C_TIMINGR        0x00110205u
#define GNSS_I2C_FLAG_TIMEOUT_MS 5u
#define GNSS_I2C_CHUNK          32u
/* Grosszuegig bemessen: nach einem Board-Reset bootet auch der Teseo neu
 * (Reset-Netz der Shields) und braucht 1..2 s, bis NMEA wieder laeuft. */
#define GNSS_COLLECT_TIMEOUT_MS 6000u
#define GNSS_BUF_LEN            512u

/* Wird true, sobald der GNSS-Treiber initialisiert ist und verwertbare Daten
 * liefern kann. */
static bool gnss_ready = false;

/* Befreit einen haengenden I2C-Bus. Wird der MCU mitten in einer laufenden
 * Uebertragung resettet, haelt der Teseo SDA auf low und wartet auf
 * Taktflanken -- das Peripheral koennte dann nie eine Start-Bedingung
 * erzeugen. Abhilfe laut I2C-Spezifikation: die Pins kurz als GPIO
 * betreiben, bis zu 9 Taktpulse auf SCL geben, bis SDA wieder frei ist,
 * und mit einer Stop-Bedingung abschliessen. */
static void gnss_i2c_bus_clear(void)
{
  GPIO_InitTypeDef gpio = { 0 };

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GNSS_SCL_PIN | GNSS_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GNSS_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(1);

  for (int i = 0; i < 9; ++i)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GNSS_SDA_PIN) == GPIO_PIN_SET)
    {
      break; /* Bus ist frei */
    }
    HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* Stop-Bedingung: SDA-Flanke low -> high, waehrend SCL high ist. */
  HAL_GPIO_WritePin(GPIOB, GNSS_SDA_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GNSS_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
}

/* Setzt I2C1 auf PB8/PB9 auf: erst einen evtl. haengenden Bus befreien,
 * dann Kernel-Takt CSI waehlen, Pins auf die Alternativfunktion schalten
 * (I2C ist Open-Drain mit Pull-ups) und das Peripheral mit dem
 * berechneten Timing einschalten. */
static app_status_t gnss_i2c_hw_init(void)
{
  RCC_PeriphCLKInitTypeDef clk = { 0 };
  GPIO_InitTypeDef gpio = { 0 };

  gnss_i2c_bus_clear();

  clk.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  clk.I2c1ClockSelection = RCC_I2C1CLKSOURCE_CSI;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }
  __HAL_RCC_I2C1_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GNSS_SCL_PIN | GNSS_SDA_PIN;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  I2C1->CR1 = 0;                     /* aus waehrend der Konfiguration */
  I2C1->TIMINGR = GNSS_I2C_TIMINGR;
  I2C1->CR1 = I2C_CR1_PE;            /* einschalten (Analogfilter default an) */
  return APP_STATUS_OK;
}

/* Wartet auf ein Statusflag des I2C1; false bei Timeout (Bus tot/haengt). */
static bool gnss_i2c_wait_flag(uint32_t flag)
{
  uint32_t start = HAL_GetTick();

  while ((I2C1->ISR & flag) == 0u)
  {
    if ((HAL_GetTick() - start) > GNSS_I2C_FLAG_TIMEOUT_MS)
    {
      return false;
    }
  }
  return true;
}

/* Setzt das I2C1-Peripheral nach einem Timeout zurueck (PE aus/ein):
 * loescht einen evtl. haengengebliebenen, angefangenen Transfer. */
static void gnss_i2c_reset_peripheral(void)
{
  I2C1->CR1 = 0;
  for (volatile int i = 0; i < 100; ++i)
  {
    __NOP(); /* PE muss einige Takte low bleiben (Referenzhandbuch) */
  }
  I2C1->CR1 = I2C_CR1_PE;
}

/* Prueft per leerem Schreibzugriff, ob der Teseo mit ACK antwortet --
 * beweist Versorgung, Verdrahtung und Adresse in einem. */
static bool gnss_i2c_probe(void)
{
  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
  I2C1->CR2 = ((uint32_t)GNSS_I2C_ADDR << 1) | I2C_CR2_AUTOEND | I2C_CR2_START;
  if (!gnss_i2c_wait_flag(I2C_ISR_STOPF))
  {
    gnss_i2c_reset_peripheral();
    return false;
  }
  bool ack = ((I2C1->ISR & I2C_ISR_NACKF) == 0u);
  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
  return ack;
}

/* Liest einen Block NMEA-Bytes vom Teseo; 0xFF-Fuellbytes werden verworfen.
 * Rueckgabe: Anzahl uebernommener Nutzbytes. Das Peripheral wickelt Start,
 * Adressierung, ACKs und Stop selbststaendig ab (AUTOEND); die CPU holt nur
 * jedes fertige Byte aus dem Empfangsregister (RXNE-Flag). */
static uint32_t gnss_i2c_read_chunk(uint8_t *dst, uint32_t max)
{
  uint32_t n = 0;

  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
  I2C1->CR2 = ((uint32_t)GNSS_I2C_ADDR << 1) | I2C_CR2_RD_WRN |
              (GNSS_I2C_CHUNK << I2C_CR2_NBYTES_Pos) |
              I2C_CR2_AUTOEND | I2C_CR2_START;
  for (uint32_t i = 0; i < GNSS_I2C_CHUNK; ++i)
  {
    if (!gnss_i2c_wait_flag(I2C_ISR_RXNE))
    {
      /* NACK oder haengender Bus: Peripheral zuruecksetzen, damit der
       * naechste Poll sauber startet; bisherige Bytes behalten. */
      I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
      gnss_i2c_reset_peripheral();
      return n;
    }
    uint8_t b = (uint8_t)I2C1->RXDR;
    if (b != 0xFFu && n < max)
    {
      dst[n++] = b;
    }
  }
  (void)gnss_i2c_wait_flag(I2C_ISR_STOPF);
  I2C1->ICR = I2C_ICR_STOPCF;
  return n;
}

/* ==========================================================================
 * Senden proprietaerer NMEA-Befehle ($PSTM...) an den Teseo.
 *
 * Bisher hat der Treiber dem Modul nur zugehoert. Zum Abfragen der
 * Firmware-Version und spaeter zum Umstellen der Fix-Rate muss er auch
 * senden koennen. Laut Datenblatt unterstuetzt der Teseo-LIV4F das
 * NMEA-Protokoll auf UART UND I2C -- es ist also derselbe Bus wie beim
 * Lesen, nur in die andere Richtung.
 * ========================================================================== */

#define GNSS_CMD_MAX_LEN     96u   /* passt in NBYTES (8 Bit) */
#define GNSS_CMD_ANSWER_MS  700u   /* Wartezeit auf die Antwort je Befehl */
#define GNSS_CMD_RETRIES      3u   /* Wiederholungen bei NACK */

/* Kein fuehrendes Port-Byte: AN5203 (ST, "Teseo-LIV3F -- I2C Positioning
 * Sensor", Abschnitt 4.4) sendet den NMEA-Rohstring unveraendert an
 * Adresse 0x3A. Das 0xFF aus der Arduino-Bibliothek stm32duino ist dort
 * eine Eigenheit der Wire-Abstraktion, keine Anforderung des Moduls. */

static const char gnss_hex[] = "0123456789ABCDEF";

/* Ergebnis eines Schreibversuchs -- absichtlich fein aufgeschluesselt,
 * weil "hat nicht geklappt" beim Bring-up nicht weiterhilft. */
typedef enum
{
  GNSS_TX_OK = 0,
  GNSS_TX_NACK_ADDR,   /* Modul quittiert die Adresse nicht */
  GNSS_TX_NACK_DATA,   /* Adresse ok, aber ein Datenbyte abgelehnt */
  GNSS_TX_TIMEOUT,     /* Bus haengt: weder TXIS noch NACK kamen */
  GNSS_TX_NO_STOP,     /* Daten raus, aber kein sauberes Stop */
} gnss_tx_result_t;

/* Wartet auf TXIS -- oder darauf, dass stattdessen ein NACK eintrifft.
 * Wichtig: Bei einem NACK setzt das Peripheral TXIS nie, ein reines
 * Warten auf TXIS wuerde also immer in den Timeout laufen und die
 * eigentliche Ursache verschleiern. */
static gnss_tx_result_t gnss_i2c_wait_txis(void)
{
  uint32_t start = HAL_GetTick();

  for (;;)
  {
    uint32_t isr = I2C1->ISR;

    if ((isr & I2C_ISR_NACKF) != 0u)
    {
      return GNSS_TX_NACK_DATA;
    }
    if ((isr & I2C_ISR_TXIS) != 0u)
    {
      return GNSS_TX_OK;
    }
    if ((HAL_GetTick() - start) > GNSS_I2C_FLAG_TIMEOUT_MS)
    {
      return GNSS_TX_TIMEOUT;
    }
  }
}

/* Schreibt einen Block auf den Bus. Gegenstueck zu gnss_i2c_read_chunk():
 * Das Peripheral wickelt Start, Adressierung und Stop selbst ab (AUTOEND),
 * die CPU schiebt nur jedes Byte ins Senderegister, sobald TXIS meldet,
 * dass Platz ist. failed_at liefert bei Fehlern den Byte-Index zurueck. */
static gnss_tx_result_t gnss_i2c_write_block(const uint8_t *src, uint32_t len,
                                             uint32_t *failed_at)
{
  gnss_tx_result_t res = GNSS_TX_OK;

  if (failed_at != NULL)
  {
    *failed_at = 0;
  }

  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
  I2C1->CR2 = ((uint32_t)GNSS_I2C_ADDR << 1) |
              (len << I2C_CR2_NBYTES_Pos) |
              I2C_CR2_AUTOEND | I2C_CR2_START;

  for (uint32_t i = 0; i < len; ++i)
  {
    res = gnss_i2c_wait_txis();
    if (res != GNSS_TX_OK)
    {
      /* Ein NACK auf das allererste Byte bedeutet: schon die Adresse
       * wurde nicht quittiert (das Modul nimmt gar keine Schreibzugriffe
       * an), spaeter heisst es: Adresse ok, Inhalt abgelehnt. */
      if (res == GNSS_TX_NACK_DATA && i == 0u)
      {
        res = GNSS_TX_NACK_ADDR;
      }
      if (failed_at != NULL)
      {
        *failed_at = i;
      }
      I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
      gnss_i2c_reset_peripheral();
      return res;
    }
    I2C1->TXDR = src[i];
  }

  if (!gnss_i2c_wait_flag(I2C_ISR_STOPF))
  {
    gnss_i2c_reset_peripheral();
    return GNSS_TX_NO_STOP;
  }
  if ((I2C1->ISR & I2C_ISR_NACKF) != 0u)
  {
    if (failed_at != NULL)
    {
      *failed_at = len;
    }
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    return GNSS_TX_NACK_DATA;
  }
  I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
  return GNSS_TX_OK;
}

/* Schreibt denselben Inhalt, aber jedes Byte als eigene I2C-Transaktion
 * (Start, Adresse, ein Byte, Stop). Testet die Vermutung, dass der Teseo
 * pro Schreibzugriff nur ein einziges Byte annimmt. */
static gnss_tx_result_t gnss_i2c_write_bytewise(const uint8_t *src, uint32_t len,
                                                uint32_t *failed_at)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    gnss_tx_result_t r = gnss_i2c_write_block(&src[i], 1u, NULL);

    if (r != GNSS_TX_OK)
    {
      if (failed_at != NULL)
      {
        *failed_at = i;
      }
      return r;
    }
  }
  return GNSS_TX_OK;
}

static const char *gnss_tx_result_text(gnss_tx_result_t res)
{
  switch (res)
  {
    case GNSS_TX_OK:        return "ok";
    case GNSS_TX_NACK_ADDR: return "NACK auf die Adresse (Modul nimmt keine Schreibzugriffe an)";
    case GNSS_TX_NACK_DATA: return "NACK auf ein Datenbyte";
    case GNSS_TX_TIMEOUT:   return "Timeout (weder TXIS noch NACK)";
    case GNSS_TX_NO_STOP:   return "kein Stop-Zustand";
    default:                return "unbekannt";
  }
}

/* STAND DER INBETRIEBNAHME (11.08.2026) -- WICHTIG:
 * Der Teseo-LIV4F lehnt Schreibzugriffe auf dem I2C bislang vollstaendig
 * ab. Am Board reproduzierbar gemessen:
 *   - leerer Schreibzugriff (nur Adressierung): wird mit ACK quittiert,
 *     Verdrahtung und Adresse 0x3A stimmen also auch beim Schreiben;
 *   - sobald ein Datenbyte folgt: NACK auf das ERSTE Byte.
 * Ausgeschlossene Ursachen (alle einzeln geprueft, Verhalten identisch):
 *   - Bustakt: 380 kHz und 100 kHz;
 *   - Inhalt des ersten Bytes: '$' wie auch das Port-Byte 0xFF;
 *   - Rahmenaufbau: ein Block mit AUTOEND wie auch jedes Byte als eigene
 *     Transaktion (gnss_i2c_write_bytewise);
 *   - Wiederholungen: 3 Versuche mit 20 ms Abstand;
 *   - Neuinitialisierung des I2C-Peripherals direkt vor dem Senden, wie
 *     sie AN5203 Abschnitt 4.4 in seiner Hauptschleife vormacht.
 *
 * WAHRSCHEINLICHE URSACHE (nicht pruefbar ohne Schreibzugriff):
 * AN5203 Abschnitt 1.1 setzt voraus, dass die Modulkonfiguration vorher
 * geaendert wurde -- die I2C-Nachrichtenliste (CDB-ID 231/232) soll auf
 * null stehen, damit das GNSS den internen I2C-Puffer nicht dauernd mit
 * Positionsdaten fuellt. Unser Modul laeuft in der Werkseinstellung und
 * streamt fortlaufend NMEA ueber I2C, also genau in dem Zustand, den die
 * Application Note ausdruecklich abschalten laesst. Diese Konfiguration
 * wird laut ST mit dem PC-Werkzeug Teseo-Suite vorgenommen, nicht ueber
 * I2C -- der Schreibkanal laesst sich also vermutlich nicht per I2C
 * selbst freischalten.
 *
 * NAECHSTER SCHRITT: UART-Port des Teseo nutzen (auf der X-NUCLEO-LIV4A1
 * herausgefuehrt) -- entweder einmalig per USB-UART-Adapter und
 * Teseo-Suite zum Umkonfigurieren, oder dauerhaft als Befehlsweg aus der
 * Firmware heraus.
 *
 * Baut aus dem Rumpf einen vollstaendigen NMEA-Satz und sendet ihn.
 * Uebergeben wird nur der Teil zwischen '$' und '*', z. B.
 * "PSTMGETSWVER,255" -- Dollarzeichen, Pruefsumme und Zeilenende ergaenzt
 * diese Funktion. Die NMEA-Pruefsumme ist das XOR aller Zeichen zwischen
 * '$' und '*', ausgegeben als zwei Hex-Ziffern. */
app_status_t gnss_send_command(const char *body)
{
  char frame[GNSS_CMD_MAX_LEN];
  uint8_t crc = 0;
  uint32_t n = 0;

  if (!gnss_ready || body == NULL)
  {
    return APP_STATUS_NOT_READY;
  }

  frame[n++] = '$';
  for (const char *p = body; *p != '\0'; ++p)
  {
    if (n >= (GNSS_CMD_MAX_LEN - 5u))
    {
      printf("[GNSS] Befehl zu lang fuer den Sendepuffer.\r\n");
      return APP_STATUS_ERROR;
    }
    crc ^= (uint8_t)*p;
    frame[n++] = *p;
  }
  frame[n++] = '*';
  frame[n++] = gnss_hex[(crc >> 4) & 0x0Fu];
  frame[n++] = gnss_hex[crc & 0x0Fu];
  frame[n++] = '\r';
  frame[n++] = '\n';

  /* I2C-Peripheral unmittelbar vor dem Senden neu aufsetzen. AN5203 macht
   * das in seiner Hauptschleife vor jeder Transaktion (HAL_I2C_DeInit +
   * HAL_I2C_Init) -- offenbar hinterlaesst der fortlaufende Lesebetrieb
   * einen Zustand, in dem das Modul Schreibzugriffe nicht annimmt. */
  if (gnss_i2c_hw_init() != APP_STATUS_OK)
  {
    printf("[GNSS] I2C liess sich vor dem Senden nicht neu aufsetzen.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Wiederholversuche, falls das Modul gerade nicht aufnahmebereit ist.
   * Der Bustakt spielt dabei keine Rolle -- 380 kHz und 100 kHz verhielten
   * sich beim Bring-up identisch. */
  uint32_t failed_at = 0;
  gnss_tx_result_t res = GNSS_TX_TIMEOUT;

  for (uint32_t attempt = 0; attempt < GNSS_CMD_RETRIES; ++attempt)
  {
    res = gnss_i2c_write_block((const uint8_t *)frame, n, &failed_at);
    if (res == GNSS_TX_OK)
    {
      break;
    }
    HAL_Delay(20);
  }

  /* Ausweichweg: einzelne Bytes je Transaktion. */
  if (res != GNSS_TX_OK)
  {
    res = gnss_i2c_write_bytewise((const uint8_t *)frame, n, &failed_at);
    if (res == GNSS_TX_OK)
    {
      printf("[GNSS] Hinweis: Modul nimmt nur ein Byte je Transaktion an.\r\n");
    }
  }

  if (res != GNSS_TX_OK)
  {
    printf("[GNSS] Befehl $%s abgewiesen: %s (Byte %lu von %lu).\r\n",
           body, gnss_tx_result_text(res),
           (unsigned long)failed_at, (unsigned long)n);
    return APP_STATUS_ERROR;
  }
  return APP_STATUS_OK;
}

/* Liest nach einem Befehl den NMEA-Strom mit und gibt jede $PSTM-Antwort
 * auf der Konsole aus. Rueckgabe: Anzahl gefundener Antwortsaetze.
 *
 * ACHTUNG: blockiert bis zum Ablauf von timeout_ms. Nur fuer Bring-up und
 * Diagnose gedacht und nur mit timeout_ms deutlich unter der Watchdog-Zeit
 * (4 s) aufrufen -- am besten aus app_init(), also bevor der Watchdog
 * scharf geschaltet wird. */
static uint32_t gnss_collect_answer(uint32_t timeout_ms)
{
  uint8_t chunk[GNSS_I2C_CHUNK];
  char line[GNSS_CMD_MAX_LEN];
  uint32_t line_len = 0;
  bool in_line = false;
  uint32_t hits = 0;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    uint32_t n = gnss_i2c_read_chunk(chunk, sizeof(chunk));

    for (uint32_t i = 0; i < n; ++i)
    {
      char c = (char)chunk[i];

      if (c == '$')
      {
        in_line = true;
        line_len = 0;
        continue;
      }
      if (!in_line)
      {
        continue;
      }
      if (c == '\r' || c == '\n')
      {
        line[line_len] = '\0';
        if (line_len > 4u && strncmp(line, "PSTM", 4) == 0)
        {
          printf("[GNSS] Antwort: $%s\r\n", line);
          ++hits;
        }
        in_line = false;
        continue;
      }
      if (line_len < (sizeof(line) - 1u))
      {
        line[line_len++] = c;
      }
    }
    HAL_Delay(2);
  }
  return hits;
}

/* Fragt die Firmware-Version des Moduls ab.
 *
 * Die Zuordnung der Abfrage-IDs unterscheidet sich zwischen den Teseo-
 * Generationen (massgeblich ist UM3009 fuer den LIV4F). Statt auf eine ID
 * zu setzen, werden mehrere gaengige abgefragt und alles ausgegeben, was
 * zurueckkommt -- fuer eine einmalige Bring-up-Abfrage der robusteste Weg. */
app_status_t gnss_query_version(void)
{
  static const char *const queries[] =
  {
    "PSTMGETSWVER,255",
    "PSTMGETSWVER,6",
    "PSTMGETSWVER,0",
  };
  uint32_t hits = 0;

  if (!gnss_ready)
  {
    return APP_STATUS_NOT_READY;
  }

  /* Vorprobe: Nimmt das Modul ueberhaupt einen Schreibzugriff an? Der
   * leere Schreibzugriff testet nur Adressierung und ACK -- schlaegt schon
   * er fehl, liegt es nicht am Inhalt des Befehls. */
  printf("[GNSS] Schreibzugriff-Vorprobe: %s\r\n",
         gnss_i2c_probe() ? "Adresse wird quittiert" : "keine Quittung");

  printf("[GNSS] frage Firmware-Version ab...\r\n");
  for (uint32_t i = 0; i < (sizeof(queries) / sizeof(queries[0])); ++i)
  {
    if (gnss_send_command(queries[i]) != APP_STATUS_OK)
    {
      continue;
    }
    hits += gnss_collect_answer(GNSS_CMD_ANSWER_MS);
  }

  if (hits == 0u)
  {
    printf("[GNSS] keine Versionsantwort erhalten -- Modul akzeptiert die "
           "Abfrage nicht oder nutzt andere IDs (siehe UM3009).\r\n");
    return APP_STATUS_ERROR;
  }
  return APP_STATUS_OK;
}

/* ==========================================================================
 * UART-Weg zum Teseo (Bring-up).
 *
 * Der UART des Moduls liegt auf der X-NUCLEO-LIV4A1 auf den Arduino-Pins
 * D0/D1 und damit auf PB7/PB6 des Nucleo -- also auf LPUART1, das CubeMX
 * ohnehin schon einrichtet (board->gnss_uart). Es braucht also keinen
 * externen USB-UART-Adapter, um Befehle ins Modul zu bekommen.
 *
 * Die Werks-Baudrate steht weder im Datenblatt des LIV4F noch in AN5203,
 * deshalb wird sie hier durchprobiert: bei welcher Rate ein '$' im Strom
 * auftaucht, die stimmt.
 * ========================================================================== */

#define GNSS_UART_LISTEN_MS   600u

/* Horcht bei der aktuellen Einstellung und zaehlt Bytes sowie '$'-Zeichen.
 * pstm != NULL: gefundene $PSTM-Antwortsaetze werden ausgegeben. */
static uint32_t gnss_uart_listen(UART_HandleTypeDef *u, uint32_t ms,
                                 uint32_t *dollars, uint32_t *pstm)
{
  char line[GNSS_CMD_MAX_LEN];
  uint32_t line_len = 0;
  bool in_line = false;
  uint32_t total = 0;
  uint32_t start = HAL_GetTick();

  if (dollars != NULL) { *dollars = 0; }
  if (pstm != NULL)    { *pstm = 0; }

  while ((HAL_GetTick() - start) < ms)
  {
    uint8_t b;

    if (HAL_UART_Receive(u, &b, 1, 5) != HAL_OK)
    {
      continue;
    }
    ++total;

    char c = (char)b;
    if (c == '$')
    {
      if (dollars != NULL) { ++(*dollars); }
      in_line = true;
      line_len = 0;
      continue;
    }
    if (!in_line)
    {
      continue;
    }
    if (c == '\r' || c == '\n')
    {
      line[line_len] = '\0';
      if (pstm != NULL && line_len > 4u && strncmp(line, "PSTM", 4) == 0)
      {
        printf("[GNSS-UART] Antwort: $%s\r\n", line);
        ++(*pstm);
      }
      in_line = false;
      continue;
    }
    if (line_len < (sizeof(line) - 1u))
    {
      line[line_len++] = c;
    }
  }
  return total;
}

/* Sucht die Baudrate des Teseo-UART und fragt dort die Firmware-Version ab.
 * Blockiert einige Sekunden -- nur aus app_init() aufrufen. */
app_status_t gnss_uart_bringup(void)
{
  static const uint32_t bauds[] = { 9600u, 19200u, 38400u, 57600u, 115200u, 230400u };
  static const char *const queries[] =
  {
    "PSTMGETSWVER,255",
    "PSTMGETSWVER,6",
    "PSTMGETSWVER,0",
  };
  const board_interfaces_t *board = board_get_interfaces();
  UART_HandleTypeDef *u = board->gnss_uart;
  uint32_t found_baud = 0;

  printf("[GNSS-UART] suche Baudrate des Teseo auf LPUART1 (PB6/PB7)...\r\n");

  for (uint32_t i = 0; i < (sizeof(bauds) / sizeof(bauds[0])); ++i)
  {
    uint32_t dollars = 0;

    u->Init.BaudRate = bauds[i];
    if (HAL_UART_Init(u) != HAL_OK)
    {
      printf("[GNSS-UART] %lu Baud: nicht einstellbar.\r\n", (unsigned long)bauds[i]);
      continue;
    }

    /* Zusaetzlich die rohe Leitungsaktivitaet zaehlen: Rahmen- und
     * Rauschfehler entstehen nur, wenn ueberhaupt Flanken ankommen.
     * Damit laesst sich "falsche Baudrate" von "gar kein Signal"
     * unterscheiden -- der entscheidende Hinweis, ob die Leitung
     * ueberhaupt verbunden ist. */
    uint32_t errors = 0;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 100u)
    {
      uint32_t isr = u->Instance->ISR;

      if ((isr & (USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE | USART_ISR_RXNE_RXFNE)) != 0u)
      {
        ++errors;
        u->Instance->ICR = USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
        (void)u->Instance->RDR;
      }
    }

    uint32_t total = gnss_uart_listen(u, GNSS_UART_LISTEN_MS, &dollars, NULL);
    printf("[GNSS-UART] %6lu Baud: %3lu Bytes, %lu x '$', Leitungsaktivitaet %lu\r\n",
           (unsigned long)bauds[i], (unsigned long)total,
           (unsigned long)dollars, (unsigned long)errors);

    if (dollars > 0u && found_baud == 0u)
    {
      found_baud = bauds[i];
    }
  }

  if (found_baud == 0u)
  {
    printf("[GNSS-UART] keine Baudrate liefert NMEA -- UART-Ausgabe des Moduls "
           "vermutlich abgeschaltet oder Pins anders belegt.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[GNSS-UART] Baudrate gefunden: %lu -- frage Firmware-Version ab.\r\n",
         (unsigned long)found_baud);
  u->Init.BaudRate = found_baud;
  (void)HAL_UART_Init(u);

  uint32_t answers = 0;
  for (uint32_t i = 0; i < (sizeof(queries) / sizeof(queries[0])); ++i)
  {
    char frame[GNSS_CMD_MAX_LEN];
    uint8_t crc = 0;
    uint32_t n = 0;
    uint32_t hits = 0;

    frame[n++] = '$';
    for (const char *p = queries[i]; *p != '\0'; ++p)
    {
      crc ^= (uint8_t)*p;
      frame[n++] = *p;
    }
    frame[n++] = '*';
    frame[n++] = gnss_hex[(crc >> 4) & 0x0Fu];
    frame[n++] = gnss_hex[crc & 0x0Fu];
    frame[n++] = '\r';
    frame[n++] = '\n';

    if (HAL_UART_Transmit(u, (uint8_t *)frame, (uint16_t)n, 500) != HAL_OK)
    {
      printf("[GNSS-UART] Senden von $%s fehlgeschlagen.\r\n", queries[i]);
      continue;
    }
    (void)gnss_uart_listen(u, GNSS_CMD_ANSWER_MS, NULL, &hits);
    answers += hits;
  }

  if (answers == 0u)
  {
    printf("[GNSS-UART] Modul sendet NMEA, antwortet aber nicht auf die "
           "Versionsabfrage.\r\n");
    return APP_STATUS_ERROR;
  }
  return APP_STATUS_OK;
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

  /* Schritt 1: I2C1-Peripheral auf PB8/PB9 aufsetzen. */
  if (gnss_i2c_hw_init() != APP_STATUS_OK)
  {
    printf("[GNSS] I2C1-Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Schritt 2: Erreichbarkeit pruefen (ACK auf die Adresse). Nach einem
   * echten Kalt-Einschalten (Powerbank!) braucht der Teseo bis zu ~2 s,
   * bis er auf I2C antwortet -- deshalb bis zu 3 s lang wiederholen
   * statt nur einmal zu fragen. */
  bool present = false;
  for (int attempt = 0; attempt < 30 && !present; ++attempt)
  {
    present = gnss_i2c_probe();
    if (!present)
    {
      HAL_Delay(100);
    }
  }
  if (!present)
  {
    printf("[GNSS] Teseo antwortet nicht auf I2C-Adresse 0x%02X.\r\n", GNSS_I2C_ADDR);
    return APP_STATUS_ERROR;
  }

  /* Schritt 3: nachweisen, dass wirklich NMEA-Saetze kommen (mindestens
   * ein "$G"-Satzanfang). Das Modul sendet im Sekundentakt; direkt nach
   * einem Board-Reset bootet es selbst neu, daher das grosse Zeitfenster. */
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

/* Verarbeitet ein einzelnes Zeichen aus dem NMEA-Strom.
 * Arbeitsweise wie eine kleine Zustandsmaschine: '$' beginnt eine neue
 * Zeile, Zeilenende (\r oder \n) schliesst sie ab und stoesst die
 * Auswertung an, alles dazwischen wird gesammelt. */
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

  /* Lesetakt begrenzen: alle 5 ms ein 32-Byte-Block (~1 ms Buszeit) leert
   * den NMEA-Strom (ca. 0,5 kB/s) locker, ohne die Hauptschleife zu bremsen. */
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
