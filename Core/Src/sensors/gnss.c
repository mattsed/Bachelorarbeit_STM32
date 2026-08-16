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

/* TIMINGR fuer 4-MHz-Kerneltakt (CSI), t_PRESC = 250 ns.
 *
 * Standard-Mode ~111 kHz: PRESC=0, SCLL=19 (5,0 us low), SCLH=15 (4,0 us
 * high), SDADEL=2, SCLDEL=4.
 *
 * WARUM langsam: Auf dem X-NUCLEO-LIV4A1 sitzen laut Schaltplan KEINE
 * I2C-Pull-ups -- die einzigen 4,7k gehoeren zu den Tastern WAKEUP und
 * nRESET. Der Bus haengt damit an den internen Pull-ups des STM32 (~40 kOhm).
 * Deren Anstiegszeit (Groessenordnung 1 us) sprengt die 300 ns, die der
 * Fast-Mode erlaubt, und lag ueber der SCL-High-Phase von 0,75 us bei den
 * frueheren 380 kHz. Standard-Mode erlaubt 1000 ns Anstiegszeit und gibt
 * dem Pegel viermal mehr Zeit -- Test vom 16.08.2026 gegen die sporadischen
 * Abrisse des NMEA-Stroms.
 * Vorheriger Wert (ca. 380 kHz, Fast-Mode): 0x00110205 */
#define GNSS_I2C_TIMINGR        0x00420F13u
#define GNSS_I2C_FLAG_TIMEOUT_MS 5u
/* Blockgroesse: Bei 100 kHz dauert ein Byte rund 90 us, ein 64-Byte-Block
 * also ~5,8 ms. Mehr darf ein Lesevorgang nicht blockieren, sonst reisst
 * die 20-ms-Messtaktung. Die 180 Byte aus STs Referenzcode (AN5203) waeren
 * hier 16 ms -- untragbar. 64 Byte alle 12 ms ergeben 5,3 kB/s und damit
 * das Sechsfache dessen, was der Teseo mit ~0,84 kB/s nachlegt. */
#define GNSS_I2C_CHUNK          64u
/* Grosszuegig bemessen: nach einem Board-Reset bootet auch der Teseo neu
 * (Reset-Netz der Shields) und braucht 1..2 s, bis NMEA wieder laeuft. */
#define GNSS_COLLECT_TIMEOUT_MS 6000u
#define GNSS_BUF_LEN            512u

/* Wird true, sobald der GNSS-Treiber initialisiert ist und verwertbare Daten
 * liefern kann. */
static bool gnss_ready = false;

/* ==========================================================================
 * Datenquelle: UART bevorzugt, I2C als Rueckfallebene
 * ==========================================================================
 * Der Teseo gibt denselben NMEA-Strom parallel ueber beide Schnittstellen
 * aus -- AN5203, Kapitel 1: "The standard NMEA over I2C interface is a
 * mirror of NMEA over UART interface".
 *
 * WARUM UART bevorzugt (Befund 16.08.2026): Der I2C-Slave des Moduls stellt
 * nach 26..138 s reproduzierbar den Betrieb ein und quittiert seine Adresse
 * nicht mehr, waehrend der GNSS-Kern normal weiterlaeuft (PPS-LED blinkt).
 * Weder Bus-Befreiung noch Peripheral-Neuaufbau noch Board-Reset holen ihn
 * zurueck -- nur ein Power-Cycle. Getestet und ausgeschlossen wurden zudem
 * Zugriffsmuster (200 -> 40 Transaktionen/s) und Busgeschwindigkeit
 * (380 -> 111 kHz). UART kennt diese Fehlerklasse nicht: kein Adressabgleich,
 * keine Slave-Zustandsmaschine, nichts, was sich verklemmen koennte.
 *
 * HARDWARE: Auf dem X-NUCLEO-LIV4A1 ist R3 in der Leitung Modul-TX ->
 * Arduino-Header unbestueckt (am Board bestaetigt). Der UART-Weg
 * funktioniert erst mit einer Bruecke von TP1 (direkt am Modulpin
 * UART-TX) auf Arduino-Pin D0 = PB7.
 */
typedef enum {
  GNSS_SRC_NONE = 0,
  GNSS_SRC_UART,
  GNSS_SRC_I2C
} gnss_source_t;

static gnss_source_t gnss_source = GNSS_SRC_NONE;
static uint32_t gnss_uart_baud = 0;

/* Empfang per Interrupt in einen Ringpuffer -- bewusst NICHT pollend:
 * Waehrend eines SD-Schreibzugriffs steht die Hauptschleife einige zehn
 * Millisekunden. Bei 9600 Baud kommt alle ~1 ms ein Byte, und das
 * Empfangsregister fasst genau eines (FIFO ist in main.c abgeschaltet).
 * Ohne Interrupt gingen die Bytes jedes Schreibvorgangs verloren. */
#define GNSS_RX_RING_LEN  512u

static volatile uint8_t  gnss_rx_ring[GNSS_RX_RING_LEN];
static volatile uint16_t gnss_rx_head = 0;   /* schreibt der Interrupt */
static volatile uint16_t gnss_rx_tail = 0;   /* liest gnss_poll()      */
static volatile uint32_t gnss_rx_lost = 0;   /* Ringpuffer war voll    */

/* Interrupt-Handler fuer LPUART1. In stm32h5xx_it.c existiert keiner --
 * der LPUART1-Interrupt war in CubeMX nie aktiviert -- daher hier. */
void LPUART1_IRQHandler(void)
{
  uint32_t isr = LPUART1->ISR;

  /* Fehlerflags zuerst quittieren: Ein stehendes Overrun-Flag blockiert
   * den weiteren Empfang. Framing-/Noise-Fehler treten bei falscher
   * Baudrate massenhaft auf und duerfen den Handler nicht aufhalten. */
  if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0u)
  {
    LPUART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  }

  while ((LPUART1->ISR & USART_ISR_RXNE) != 0u)
  {
    uint8_t b = (uint8_t)LPUART1->RDR;
    uint16_t next = (uint16_t)((gnss_rx_head + 1u) % GNSS_RX_RING_LEN);

    if (next != gnss_rx_tail)
    {
      gnss_rx_ring[gnss_rx_head] = b;
      gnss_rx_head = next;
    }
    else
    {
      ++gnss_rx_lost;   /* Puffer voll -- Byte verwerfen, nie blockieren */
    }
  }
}

/* Abriss-Erkennung: Der Teseo sendet seine NMEA-Buendel im Sekundentakt;
 * dazwischen liefert er nur 0xFF-Fuellbytes (die read_chunk verwirft).
 * Mehrere Sekunden ohne ein einziges Nutzbyte bedeuten deshalb: Verbindung
 * tot. Beobachtet am 16.08.2026 (LOG_034): I2C-Strom riss nach 26 s ab,
 * waehrend das Modul selbst weiterlief (PPS-LED blinkte weiter) -- ohne
 * diese Erkennung blieb fix_valid einfach auf dem letzten Stand stehen und
 * die CSV meldete 4 Minuten lang eine eingefrorene Position als gueltig. */
#define GNSS_STREAM_TIMEOUT_MS  3000u
#define GNSS_RECOVER_PERIOD_MS  5000u

static uint32_t gnss_last_rx_ms = 0;      /* letztes echtes Nutzbyte */
static uint32_t gnss_next_recover_ms = 0; /* naechster Wiederbelebungsversuch */
static bool gnss_stream_lost = false;     /* Meldung nur auf der Flanke */

/* Diagnose des Abrisses: Lief die letzte Lesetransaktion sauber durch (dann
 * quittiert das Modul weiter, liefert aber nur 0xFF-Fuellbytes -- der
 * GNSS-Kern fuellt den I2C-Puffer nicht mehr), oder brach sie mit NACK bzw.
 * Timeout ab (dann ist der I2C-Slave des Moduls selbst weg)? Genau diese
 * Unterscheidung sagt uns, wo der Fehler sitzt. */
static bool gnss_rx_bus_ok = false;
static uint32_t gnss_recover_tries = 0;

/* true, wenn der zuletzt gelesene Block auf einem 0xFF-Fuellbyte endete --
 * nach STs Kriterium (AN5203) ist der I2C-Puffer des Moduls dann leer und
 * weiteres Lesen bringt nichts mehr. */
static bool gnss_rx_drained = false;

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

  /* IMMER volle 9 Taktpulse geben, auch wenn SDA zwischendurch high ist:
   * Der Teseo kann mitten in einem Byte stehen und gerade zufaellig eine 1
   * ausgeben; ein vorzeitiger Abbruch laesst ihn dann zwischen zwei Bits
   * stehen. Bis zu drei Runden, falls SDA danach immer noch festgehalten
   * wird.
   *
   * HINWEIS 16.08.2026: Das ist spezifikationskonformer als der frueher hier
   * stehende vorzeitige Abbruch, loest aber NICHT den beobachteten Fall,
   * dass der Teseo nach einem Flash-Vorgang (STM32_Programmer_CLI mode=UR)
   * gar nicht mehr auf seine Adresse antwortet. Dort haengt das Modul
   * selbst, nicht der Bus -- es hilft nur ein Power-Cycle (USB abziehen).
   * Ein normaler Reset (mode=HOTPLUG -hardRst) ist dagegen unkritisch. */
  for (int round = 0; round < 3; ++round)
  {
    for (int i = 0; i < 9; ++i)
    {
      HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GNSS_SCL_PIN, GPIO_PIN_SET);
      HAL_Delay(1);
    }
    if (HAL_GPIO_ReadPin(GPIOB, GNSS_SDA_PIN) == GPIO_PIN_SET)
    {
      break; /* Bus ist frei */
    }
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
  uint8_t last = 0;

  gnss_rx_bus_ok = false;
  gnss_rx_drained = false;
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
    last = b;
    if (b != 0xFFu && n < max)
    {
      dst[n++] = b;
    }
  }
  (void)gnss_i2c_wait_flag(I2C_ISR_STOPF);
  I2C1->ICR = I2C_ICR_STOPCF;
  gnss_rx_bus_ok = true;                 /* alle Bytes sauber uebertragen */
  gnss_rx_drained = (last == 0xFFu);     /* Block endete auf Fuellbyte */
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

/* Setzt LPUART1 auf die gewuenschte Baudrate und schaltet den
 * Empfangs-Interrupt scharf. Der Ringpuffer wird dabei geleert. */
static app_status_t gnss_uart_start(uint32_t baud)
{
  const board_interfaces_t *board = board_get_interfaces();
  UART_HandleTypeDef *u = board->gnss_uart;

  HAL_NVIC_DisableIRQ(LPUART1_IRQn);

  u->Init.BaudRate = baud;
  if (HAL_UART_Init(u) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  gnss_rx_head = 0;
  gnss_rx_tail = 0;
  gnss_rx_lost = 0;

  LPUART1->CR1 |= USART_CR1_RXNEIE;
  /* Prioritaet unterhalb der HAL-Tickinterrupts, aber hoch genug, um
   * waehrend blockierender SPI-/SD-Zugriffe zuverlaessig zu greifen. */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  return APP_STATUS_OK;
}

/* Horcht eine Zeit lang auf der eingestellten Baudrate und zaehlt, was im
 * Ringpuffer landet. Rueckgabe: Anzahl '$' -- ab zwei Satzanfaengen gilt
 * die Baudrate als gefunden (Rauschen erzeugt praktisch nie zwei davon). */
static uint32_t gnss_uart_collect(uint32_t ms, uint32_t *bytes)
{
  uint32_t dollars = 0;
  uint32_t total = 0;

  HAL_Delay(ms);

  while (gnss_rx_tail != gnss_rx_head)
  {
    char c = (char)gnss_rx_ring[gnss_rx_tail];
    gnss_rx_tail = (uint16_t)((gnss_rx_tail + 1u) % GNSS_RX_RING_LEN);
    ++total;
    if (c == '$')
    {
      ++dollars;
    }
  }
  if (bytes != NULL)
  {
    *bytes = total;
  }
  return dollars;
}

/* Sucht die Baudrate des Teseo. Reihenfolge nach Wahrscheinlichkeit:
 * 9600 ist die Werkseinstellung der Teseo-Familie, 115200 die haeufigste
 * Abweichung. Blockiert im schlimmsten Fall rund 8 s -- nur aus
 * gnss_init() heraus aufrufen, also bevor der Watchdog scharf ist. */
static bool gnss_uart_detect(void)
{
  /* 115200 zuerst: Das ist die Werkseinstellung des Teseo-LIV4F auf dem
   * X-NUCLEO-LIV4A1 (gemessen 16.08.2026). Damit ist der Start nach 1,3 s
   * durch statt nach 2,6 s. */
  static const uint32_t bauds[] =
      { 115200u, 9600u, 19200u, 38400u, 57600u, 230400u };

  printf("[GNSS] suche NMEA auf LPUART1 (PB7)...\r\n");

  for (uint32_t i = 0; i < (sizeof(bauds) / sizeof(bauds[0])); ++i)
  {
    uint32_t bytes = 0;

    if (gnss_uart_start(bauds[i]) != APP_STATUS_OK)
    {
      continue;
    }
    uint32_t dollars = gnss_uart_collect(1300u, &bytes);
    printf("[GNSS]   %6lu Baud: %4lu Bytes, %lu x '$'\r\n",
           (unsigned long)bauds[i], (unsigned long)bytes,
           (unsigned long)dollars);

    if (dollars >= 2u)
    {
      gnss_uart_baud = bauds[i];
      printf("[GNSS] NMEA ueber UART bei %lu Baud -- diese Quelle wird genutzt.\r\n",
             (unsigned long)bauds[i]);
      return true;
    }
  }

  HAL_NVIC_DisableIRQ(LPUART1_IRQn);
  printf("[GNSS] kein NMEA auf UART (Bruecke TP1 -> D0 gelegt?) -- "
         "falle auf I2C zurueck.\r\n");
  return false;
}

app_status_t gnss_init(void)
{
  static uint8_t buf[GNSS_BUF_LEN];
  uint32_t len = 0;
  bool has_nmea = false;

  gnss_ready = false;
  gnss_source = GNSS_SRC_NONE;

  /* Schritt 0: UART bevorzugen -- siehe Begruendung oben bei gnss_source_t.
   * Nur wenn dort nichts ankommt (Bruecke TP1 -> D0 fehlt), wird der
   * bisherige I2C-Weg genommen. */
  if (gnss_uart_detect())
  {
    gnss_source = GNSS_SRC_UART;
    gnss_ready = true;
    gnss_stream_lost = false;
    gnss_last_rx_ms = HAL_GetTick();
    return APP_STATUS_OK;
  }

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
  gnss_source = GNSS_SRC_I2C;
  gnss_ready = true;
  gnss_stream_lost = false;
  gnss_last_rx_ms = HAL_GetTick();
  return APP_STATUS_OK;
}

/* ==========================================================================
 * NMEA-Parser: sammelt Zeichen aus dem I2C-Strom zu Zeilen und wertet
 * RMC-Saetze aus (Position, Geschwindigkeit, UTC-Zeit, Fix-Status).
 * ========================================================================== */

#define GNSS_LINE_LEN     100u

/* Lesetakt und Drain-Tiefe bei 100 kHz: ein 64-Byte-Block blockiert ~5,8 ms,
 * daher nur EIN Block je Runde -- so bleibt die Schleife innerhalb der
 * 20-ms-Messtaktung bedienbar. Alle 12 ms ergibt das 5,3 kB/s; ein
 * 1-Hz-Buendel von rund 840 Byte ist nach etwa 13 Runden (160 ms) abgeholt. */
#define GNSS_POLL_GAP_MS    12u
#define GNSS_DRAIN_BLOCKS   1u

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

/* Ratenbegrenzter Wiederbelebungsversuch: Bus befreien, I2C1 neu aufsetzen,
 * Modul anpingen. true = Modul quittiert seine Adresse wieder. Blockiert im
 * Fehlerfall einige Millisekunden (Bus-Clear-Taktpulse) -- bei einem alle
 * 5 s stattfindenden Versuch kostet das hoechstens vereinzelte Samples. */
typedef enum {
  GNSS_REC_SKIPPED = 0,  /* Ratenbegrenzung -- gar nicht erst versucht */
  GNSS_REC_FAILED,       /* versucht, Modul quittiert seine Adresse nicht */
  GNSS_REC_OK            /* Adresse wird wieder quittiert */
} gnss_recover_result_t;

static gnss_recover_result_t gnss_try_recover(uint32_t now)
{
  if (now < gnss_next_recover_ms)
  {
    return GNSS_REC_SKIPPED;
  }
  gnss_next_recover_ms = now + GNSS_RECOVER_PERIOD_MS;
  ++gnss_recover_tries;

  if (gnss_source == GNSS_SRC_UART)
  {
    /* Beim UART gibt es keine Gegenstelle, die man anpingen koennte --
     * hier hilft nur, das Peripheral neu aufzusetzen (etwa nach einem
     * stehengebliebenen Fehlerflag). Ob wieder Daten kommen, zeigt erst
     * der naechste Byteempfang. */
    return (gnss_uart_start(gnss_uart_baud) == APP_STATUS_OK)
             ? GNSS_REC_OK : GNSS_REC_FAILED;
  }

  if ((gnss_i2c_hw_init() == APP_STATUS_OK) && gnss_i2c_probe())
  {
    return GNSS_REC_OK;
  }
  return GNSS_REC_FAILED;
}

app_status_t gnss_poll(void)
{
  static uint8_t chunk[GNSS_I2C_CHUNK];
  uint32_t now = HAL_GetTick();

  /* Modul war beim Start nicht erreichbar (oder ist endgueltig verloren
   * gemeldet): trotzdem periodisch weiterprobieren -- ein spaeter dazu-
   * gestecktes oder wieder aufgewachtes Modul wird so doch noch gefunden. */
  if (!gnss_ready)
  {
    if (gnss_try_recover(now) == GNSS_REC_OK)
    {
      printf("[GNSS] Modul erreichbar -- Empfang laeuft an.\r\n");
      gnss_ready = true;
      gnss_stream_lost = false;
      gnss_last_rx_ms = now;
      gnss_line_len = 0;
    }
    return APP_STATUS_NOT_READY;
  }

  uint32_t n = 0;

  if (gnss_source == GNSS_SRC_UART)
  {
    /* Der Interrupt hat schon gesammelt -- hier nur den Ringpuffer in den
     * Parser leeren. Kostet keine Buszeit und blockiert nichts, deshalb
     * jede Schleifenrunde statt im 12-ms-Takt. */
    while (gnss_rx_tail != gnss_rx_head)
    {
      gnss_feed_char((char)gnss_rx_ring[gnss_rx_tail]);
      gnss_rx_tail = (uint16_t)((gnss_rx_tail + 1u) % GNSS_RX_RING_LEN);
      ++n;
    }
  }
  else
  {
    if ((now - gnss_last_poll_ms) < GNSS_POLL_GAP_MS)
    {
      return APP_STATUS_OK;
    }
    gnss_last_poll_ms = now;

    /* Lesemuster nach AN5203: Bloecke holen, bis das Modul nur noch
     * Fuellbytes liefert. Nach hoechstens GNSS_DRAIN_BLOCKS Bloecken wird
     * abgebrochen, damit ein einzelner Durchlauf die 20-ms-Messtaktung
     * nicht sprengt -- der Rest kommt in der naechsten Runde. */
    for (uint32_t block = 0; block < GNSS_DRAIN_BLOCKS; ++block)
    {
      uint32_t got = gnss_i2c_read_chunk(chunk, sizeof(chunk));
      for (uint32_t i = 0; i < got; ++i)
      {
        gnss_feed_char((char)chunk[i]);
      }
      n += got;
      if (!gnss_rx_bus_ok || gnss_rx_drained)
      {
        break;  /* Transaktion abgebrochen oder Puffer leer */
      }
    }
  }

  if (n > 0u)
  {
    gnss_last_rx_ms = now;
    if (gnss_stream_lost)
    {
      gnss_stream_lost = false;
      printf("[GNSS] NMEA-Strom wieder da.\r\n");
    }
    /* Geparst wurde bereits blockweise in der Leseschleife oben. */
    return APP_STATUS_OK;
  }

  /* Kein Nutzbyte -- zwischen zwei 1-Hz-Buendeln normal. Erst nach mehreren
   * Sekunden Stille gilt die Verbindung als abgerissen. */
  if ((now - gnss_last_rx_ms) > GNSS_STREAM_TIMEOUT_MS)
  {
    if (!gnss_stream_lost)
    {
      gnss_stream_lost = true;
      /* Sofort als ungueltig markieren: CSV-Spalte fix und gelbe LED
       * duerfen keinen veralteten Fix weitermelden. Halb gesammelte
       * NMEA-Zeile verwerfen, damit sie nicht mit dem spaeteren, mitten
       * im Satz einsetzenden Strom verklebt. */
      gnss_last.fix_valid = false;
      gnss_line_len = 0;
      gnss_recover_tries = 0;
      if (gnss_source == GNSS_SRC_UART)
      {
        printf("[GNSS] NMEA-Strom abgerissen (>%lu ms nichts) -- fix=0. "
               "Quelle UART, %lu Byte im Ringpuffer verloren.\r\n",
               (unsigned long)GNSS_STREAM_TIMEOUT_MS,
               (unsigned long)gnss_rx_lost);
      }
      else
      {
        printf("[GNSS] NMEA-Strom abgerissen (>%lu ms nichts) -- fix=0. "
               "Lesezugriff: %s\r\n",
               (unsigned long)GNSS_STREAM_TIMEOUT_MS,
               gnss_rx_bus_ok ? "quittiert, aber nur 0xFF-Fuellbytes"
                              : "bricht ab (NACK/Timeout)");
      }
    }
    /* WICHTIG: nur auf einen tatsaechlich durchgefuehrten Versuch reagieren.
     * Ein frueher hier stehendes "else" traf auch den ratenbegrenzten Fall
     * und gab die Meldung in JEDER Schleifenrunde aus -- der printf-Sturm
     * hat die 50-Hz-Abtastung lahmgelegt. */
    gnss_recover_result_t rec = gnss_try_recover(now);
    if (rec == GNSS_REC_OK)
    {
      /* Adresse wird wieder quittiert: Zaehler neu aufziehen und auf den
       * naechsten NMEA-Burst warten. Kommt keiner, meldet der Timeout den
       * Abriss erneut und der naechste Versuch folgt. */
      gnss_last_rx_ms = now;
      printf("[GNSS] I2C wiederhergestellt (Versuch %lu) -- warte auf NMEA.\r\n",
             (unsigned long)gnss_recover_tries);
    }
    else if ((rec == GNSS_REC_FAILED) && ((gnss_recover_tries % 12u) == 1u))
    {
      /* Bei 5 s Versuchsabstand etwa jede Minute ein Lebenszeichen. */
      printf("[GNSS] Wiederbelebung erfolglos (Versuch %lu, Modul quittiert "
             "seine Adresse nicht).\r\n", (unsigned long)gnss_recover_tries);
    }
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

bool gnss_has_fix(void)
{
  return gnss_ready && gnss_last.fix_valid;
}

/* ==========================================================================
 * Diagnose: rohen NMEA-Strom mitlesen
 * ==========================================================================
 * Hintergrund: gnss_feed_char() wertet AUSSCHLIESSLICH RMC-Saetze aus.
 * Fehlt RMC im Ausgabeprofil des Moduls, bleiben fix, lat, lon und v
 * dauerhaft 0 -- auch dann, wenn das Modul einen gueltigen Fix hat und die
 * rote PPS-LED auf dem Shield blinkt. Dieser Mitschnitt zeigt, welche
 * Satzarten wirklich ankommen und ob darin ein Fix gemeldet wird.
 */
app_status_t gnss_dump_stream(uint32_t duration_ms)
{
  char line[GNSS_LINE_LEN];
  static uint8_t chunk[GNSS_I2C_CHUNK];
  uint32_t len = 0;
  uint32_t lines = 0;
  uint32_t n_rmc = 0, n_gga = 0, n_gsa = 0, n_gsv = 0, n_other = 0;

  if (!gnss_ready)
  {
    printf("[GNSS-Dump] Modul nicht bereit -- nichts mitzulesen.\r\n");
    return APP_STATUS_NOT_READY;
  }

  printf("[GNSS-Dump] lese %lu ms lang den rohen NMEA-Strom mit...\r\n",
         (unsigned long)duration_ms);

  uint32_t bytes_total = 0;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < duration_ms)
  {
    uint32_t n = 0;

    /* Quelle beachten: Beim UART hat der Interrupt schon gesammelt, beim
     * I2C muss aktiv gelesen werden. */
    if (gnss_source == GNSS_SRC_UART)
    {
      while ((gnss_rx_tail != gnss_rx_head) && (n < GNSS_I2C_CHUNK))
      {
        chunk[n++] = gnss_rx_ring[gnss_rx_tail];
        gnss_rx_tail = (uint16_t)((gnss_rx_tail + 1u) % GNSS_RX_RING_LEN);
      }
    }
    else
    {
      n = gnss_i2c_read_chunk(chunk, sizeof(chunk));
    }
    bytes_total += n;

    for (uint32_t i = 0; i < n; ++i)
    {
      char c = (char)chunk[i];

      if (c == '$')
      {
        len = 0;
      }
      if (c == '\r' || c == '\n')
      {
        if (len > 6u && line[0] == '$')
        {
          line[len] = '\0';
          printf("  %s\r\n", line);
          ++lines;
          if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C')      { ++n_rmc; }
          else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A') { ++n_gga; }
          else if (line[3] == 'G' && line[4] == 'S' && line[5] == 'A') { ++n_gsa; }
          else if (line[3] == 'G' && line[4] == 'S' && line[5] == 'V') { ++n_gsv; }
          else                                                          { ++n_other; }
        }
        len = 0;
        continue;
      }
      if (len < GNSS_LINE_LEN - 1u)
      {
        line[len++] = c;
      }
    }
    HAL_Delay(GNSS_POLL_GAP_MS);
  }

  printf("[GNSS-Dump] %lu Saetze: RMC=%lu GGA=%lu GSA=%lu GSV=%lu sonstige=%lu\r\n",
         (unsigned long)lines, (unsigned long)n_rmc, (unsigned long)n_gga,
         (unsigned long)n_gsa, (unsigned long)n_gsv, (unsigned long)n_other);

  /* Datenrate mitprotokollieren: Wird die Leitungskapazitaet knapp, gehen
   * Saetze verloren, sobald mehr Satelliten sichtbar werden (mehr GSV).
   * Bei 115200 Baud (~11520 Byte/s) und gemessenen ~400 Byte/s ist davon
   * weit und breit nichts zu sehen. */
  if (gnss_source == GNSS_SRC_UART)
  {
    printf("[GNSS-Dump] %lu Byte in %lu ms = %lu Byte/s (UART %lu Baud "
           "= max. %lu Byte/s)\r\n",
           (unsigned long)bytes_total, (unsigned long)duration_ms,
           (unsigned long)((bytes_total * 1000u) / (duration_ms ? duration_ms : 1u)),
           (unsigned long)gnss_uart_baud, (unsigned long)(gnss_uart_baud / 10u));
  }
  else
  {
    printf("[GNSS-Dump] %lu Byte in %lu ms = %lu Byte/s (I2C)\r\n",
           (unsigned long)bytes_total, (unsigned long)duration_ms,
           (unsigned long)((bytes_total * 1000u) / (duration_ms ? duration_ms : 1u)));
  }

  if (gnss_rx_lost != 0u)
  {
    printf("[GNSS-Dump] ACHTUNG: %lu Byte im Ringpuffer verloren.\r\n",
           (unsigned long)gnss_rx_lost);
  }

  if (n_rmc == 0u)
  {
    printf("[GNSS-Dump] KEIN RMC empfangen -- der Parser wertet nur RMC aus, "
           "deshalb bleiben fix/lat/lon/v auf 0.\r\n");
  }
  return APP_STATUS_OK;
}
