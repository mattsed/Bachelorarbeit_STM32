#include "comms/ble_bluenrg.h"

#include <stdio.h>

#include "board/board.h"

/* BlueNRG-2 SPI-Transportprotokoll (DTM-Firmware auf dem BlueNRG-M2SP):
 * Jede Transaktion beginnt mit einem 5-Byte-Header. Das Modul antwortet im
 * ersten Byte mit 0x02 ("ready") und meldet in den Folgebytes seine
 * Puffergroessen. Der IRQ-Pin geht auf high, sobald Daten zum Abholen liegen. */
#define BLE_SPI_HDR_WRITE     0x0Au
#define BLE_SPI_HDR_READ      0x0Bu
#define BLE_SPI_READY         0x02u

#define BLE_BOOT_TIMEOUT_MS   2000u
#define BLE_EVENT_TIMEOUT_MS  1000u
#define BLE_MAX_EVENT_LEN     32u

/* Wird true, sobald das BlueNRG-Modul initialisiert und bereit zum Senden ist. */
static bool ble_bluenrg_ready = false;

static void ble_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->ble_cs.port, board->ble_cs.pin, state);
}

/* Letzter empfangener SPI-Header, fuer Diagnoseausgaben im Fehlerfall. */
static uint8_t ble_last_hdr[5];

/* SPI1 fuer den BlueNRG-2 umkonfigurieren: Mode 1 (CPOL=0, CPHA=2EDGE),
 * wie in STs X-CUBE-BLE2-Referenz, und ~1 MHz Takt (250 MHz / 256). */
static app_status_t ble_spi_config(const board_interfaces_t *board)
{
  if (HAL_SPI_DeInit(board->ble_spi) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }
  board->ble_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
  board->ble_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
  board->ble_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  /* H5-Eigenheit: ohne KeepIOState schweben SCK/MOSI zwischen den
   * Transfers (Hi-Z) und erzeugen beim Start Stoerflanken am Slave. */
  board->ble_spi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  if (HAL_SPI_Init(board->ble_spi) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  /* Diagnose: Pull-down auf MISO (PG9). Bleibt der Header-Empfang bei 0xFF,
   * wird die Leitung aktiv hochgetrieben; wird er 0x00, ist sie unterbrochen. */
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOG, &gpio);

  return APP_STATUS_OK;
}

static bool ble_irq_pending(const board_interfaces_t *board)
{
  return HAL_GPIO_ReadPin(board->ble_irq.port, board->ble_irq.pin) == GPIO_PIN_SET;
}

static bool ble_wait_irq(const board_interfaces_t *board, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (!ble_irq_pending(board))
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return false;
    }
  }
  return true;
}

/* Liest ein anstehendes Ereignis (Ablauf wie in STs hci_tl_SPI.c fuer den
 * BlueNRG-2). Rueckgabe: Anzahl Bytes oder -1. */
static int ble_spi_read(const board_interfaces_t *board, uint8_t *buf, uint16_t maxlen)
{
  uint8_t hdr_tx[5] = { BLE_SPI_HDR_READ, 0, 0, 0, 0 };
  uint8_t hdr_rx[5] = { 0 };
  uint8_t zero = 0x00u;
  int n = -1;

  /* Lese-Header austauschen: Die MCU sendet 0x0B + 4 Nullbytes, das Modul
   * antwortet gleichzeitig mit seinem Status -- in Byte 3/4 steht, wie
   * viele Ereignis-Bytes zum Abholen bereitliegen. */
  ble_cs(board, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(board->ble_spi, hdr_tx, hdr_rx, 5, 100);
  for (int i = 0; i < 5; ++i)
  {
    ble_last_hdr[i] = hdr_rx[i];
  }

  uint16_t avail = (uint16_t)(hdr_rx[3] | (hdr_rx[4] << 8));
  /* Liegen Daten an (0xFFFF = Modul hat gar nicht geantwortet), werden sie
   * Byte fuer Byte abgeholt -- CS bleibt dabei durchgehend low, denn die
   * Transaktion endet erst mit dem CS-Anheben weiter unten. */
  if (avail > 0u && avail != 0xFFFFu)
  {
    uint16_t count = (avail < maxlen) ? avail : maxlen;
    for (uint16_t i = 0; i < count; ++i)
    {
      (void)HAL_SPI_TransmitReceive(board->ble_spi, &zero, &buf[i], 1, 100);
    }
    n = (int)count;
  }

  /* Laut ST-Referenz: warten, bis der IRQ wieder faellt, dann CS anheben. */
  uint32_t start = HAL_GetTick();
  while (ble_irq_pending(board) && (HAL_GetTick() - start) < BLE_EVENT_TIMEOUT_MS)
  {
  }
  ble_cs(board, GPIO_PIN_SET);
  return n;
}

/* Sendet ein HCI-Kommando (Ablauf wie in STs hci_tl_SPI.c: nach CS-Low
 * zieht der BlueNRG-2 den IRQ hoch, sobald er bereit ist). */
static app_status_t ble_spi_write(const board_interfaces_t *board, const uint8_t *data, uint16_t len)
{
  uint8_t hdr_tx[5] = { BLE_SPI_HDR_WRITE, 0, 0, 0, 0 };
  uint8_t hdr_rx[5] = { 0 };
  uint32_t start = HAL_GetTick();

  /* Schreiben laeuft als Wiederholschleife: CS anlegen, auf Bereitschaft
   * warten, Header tauschen -- meldet das Modul zu wenig Pufferplatz,
   * CS wieder anheben und den ganzen Versuch wiederholen (bis Timeout). */
  for (;;)
  {
    ble_cs(board, GPIO_PIN_RESET);

    /* Warten, bis das Modul bereit ist (IRQ geht hoch). */
    uint32_t t0 = HAL_GetTick();
    while (!ble_irq_pending(board))
    {
      if ((HAL_GetTick() - t0) > 100u)
      {
        break;
      }
    }

    (void)HAL_SPI_TransmitReceive(board->ble_spi, hdr_tx, hdr_rx, 5, 100);
    for (int i = 0; i < 5; ++i)
    {
      ble_last_hdr[i] = hdr_rx[i];
    }

    /* Wie in STs Referenz zaehlt nur die gemeldete Puffergroesse; das erste
     * Headerbyte ist beim Aufwachen des Moduls nicht immer 0x02. */
    uint16_t wspace = (uint16_t)(hdr_rx[1] | (hdr_rx[2] << 8));
    if (wspace >= len && wspace != 0xFFFFu)
    {
      (void)HAL_SPI_Transmit(board->ble_spi, (uint8_t *)data, len, 100);
      ble_cs(board, GPIO_PIN_SET);
      return APP_STATUS_OK;
    }

    ble_cs(board, GPIO_PIN_SET);
    if ((HAL_GetTick() - start) > BLE_EVENT_TIMEOUT_MS)
    {
      return APP_STATUS_ERROR;
    }
    HAL_Delay(1);
  }
}

static void ble_print_event(const char *label, const uint8_t *buf, int len)
{
  printf("[BLE] %s:", label);
  for (int i = 0; i < len; ++i)
  {
    printf(" %02X", buf[i]);
  }
  printf("\r\n");
}

app_status_t ble_bluenrg_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  uint8_t evt[BLE_MAX_EVENT_LEN];
  int n;

  ble_bluenrg_ready = false;

  if (ble_spi_config(board) != APP_STATUS_OK)
  {
    printf("[BLE] SPI1-Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  /* IRQ-Pin mit Pull-down, damit ein schwebender Eingang nicht faelschlich
   * als "IRQ aktiv" gelesen wird (Boot-Pegel des Moduls ist definiert low). */
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = board->ble_irq.pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(board->ble_irq.port, &gpio);

  /* Definierte Ausgangslage, dann Hardware-Reset des Moduls. */
  ble_cs(board, GPIO_PIN_SET);
  HAL_GPIO_WritePin(board->ble_reset.port, board->ble_reset.pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(board->ble_reset.port, board->ble_reset.pin, GPIO_PIN_SET);

  /* Nach dem Boot meldet die DTM-Firmware das Vendor-Event "Blue Initialized"
   * (04 FF 03 01 00 01) und signalisiert es ueber den IRQ-Pin. */
  if (!ble_wait_irq(board, BLE_BOOT_TIMEOUT_MS))
  {
    printf("[BLE] kein IRQ nach Reset -- Modul antwortet nicht.\r\n");
    return APP_STATUS_ERROR;
  }

  n = ble_spi_read(board, evt, sizeof(evt));
  if (n < 0)
  {
    ble_print_event("IRQ aktiv, aber kein Ereignis lesbar; letzter Header", ble_last_hdr, 5);
    return APP_STATUS_ERROR;
  }
  ble_print_event("Boot-Event", evt, n);

  if (n < 6 || evt[0] != 0x04u || evt[1] != 0xFFu || evt[3] != 0x01u || evt[4] != 0x00u)
  {
    printf("[BLE] unerwartetes Boot-Event.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Gegenrichtung testen: HCI_RESET senden, Command-Complete-Event erwarten
   * (04 0E 04 01 03 0C 00; letztes Byte 0x00 = Status OK). */
  const uint8_t hci_reset[] = { 0x01u, 0x03u, 0x0Cu, 0x00u };
  if (ble_spi_write(board, hci_reset, sizeof(hci_reset)) != APP_STATUS_OK)
  {
    printf("[BLE] HCI_RESET liess sich nicht senden.\r\n");
    return APP_STATUS_ERROR;
  }

  if (!ble_wait_irq(board, BLE_EVENT_TIMEOUT_MS))
  {
    printf("[BLE] keine Antwort auf HCI_RESET.\r\n");
    return APP_STATUS_ERROR;
  }
  n = ble_spi_read(board, evt, sizeof(evt));
  if (n < 0)
  {
    printf("[BLE] Antwort auf HCI_RESET nicht lesbar.\r\n");
    return APP_STATUS_ERROR;
  }
  ble_print_event("HCI_RESET-Antwort", evt, n);

  /* Nach HCI_RESET startet die Firmware neu und meldet erneut "Blue
   * Initialized"; das Event kann direkt hinter dem Command Complete liegen. */
  if (n < 7 || evt[0] != 0x04u || evt[1] != 0x0Eu || evt[6] != 0x00u)
  {
    printf("[BLE] unerwartete Antwort auf HCI_RESET.\r\n");
    return APP_STATUS_ERROR;
  }

  printf("[BLE] BlueNRG-2 erkannt, Kommunikation in beide Richtungen ok.\r\n");
  ble_bluenrg_ready = true;
  return APP_STATUS_OK;
}

app_status_t ble_bluenrg_poll(void)
{
  /* Spaeter: BLE-Ereignisse abarbeiten, ohne die Messschleife zu blockieren. */
  return ble_bluenrg_ready ? APP_STATUS_OK : APP_STATUS_NOT_READY;
}

app_status_t ble_bluenrg_send_sample(const app_sample_t *sample)
{
  /* Spaeter: den aktuellen Messdatensatz als BLE-Paket oder Stream senden. */
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool ble_bluenrg_is_ready(void)
{
  return ble_bluenrg_ready;
}
