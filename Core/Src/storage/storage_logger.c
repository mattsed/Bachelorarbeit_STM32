#include "storage/storage_logger.h"

#include <stdio.h>

#include "board/board.h"

/* SD-Karten-Kommandos (SPI-Modus, vgl. SD Physical Layer Spec). */
#define SD_CMD0_GO_IDLE          0u
#define SD_CMD8_SEND_IF_COND     8u
#define SD_CMD17_READ_SINGLE     17u
#define SD_CMD55_APP_CMD         55u
#define SD_CMD58_READ_OCR        58u
#define SD_ACMD41_SD_SEND_OP     41u

/* R1-Antwortbits. */
#define SD_R1_IDLE               0x01u
#define SD_R1_ILLEGAL_CMD        0x04u

#define SD_BLOCK_SIZE            512u
#define SD_ACMD41_TIMEOUT_MS     1000u
#define SD_TOKEN_TIMEOUT_MS      200u
#define SD_DATA_TOKEN            0xFEu

/* Erkannter Kartentyp, nur fuer Diagnoseausgaben. */
typedef enum
{
  SD_CARD_NONE = 0,
  SD_CARD_V1,
  SD_CARD_V2_SC,
  SD_CARD_V2_HC,
} sd_card_type_t;

/* Wird true, sobald microSD/FatFS initialisiert ist und geschrieben werden kann. */
static bool storage_logger_ready = false;

static void sd_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->microsd_cs.port, board->microsd_cs.pin, state);
}

/* Tauscht genau ein Byte auf SPI5 aus (0xFF senden = Bus idle halten). */
static uint8_t sd_xfer(const board_interfaces_t *board, uint8_t out)
{
  uint8_t in = 0xFFu;

  (void)HAL_SPI_TransmitReceive(board->microsd_spi, &out, &in, 1, 100);
  return in;
}

/* Stellt SPI5-Taktquelle und -Vorteiler um. Die SD-Initialisierung verlangt
 * 100..400 kHz; PCLK3 (250 MHz) kann das selbst mit Vorteiler 256 nicht,
 * deshalb wird fuer die Init-Phase auf CSI (4 MHz) gewechselt. */
static app_status_t sd_spi_config(const board_interfaces_t *board, uint32_t clk_source, uint32_t prescaler)
{
  RCC_PeriphCLKInitTypeDef clk = { 0 };

  if (HAL_SPI_DeInit(board->microsd_spi) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  clk.PeriphClockSelection = RCC_PERIPHCLK_SPI5;
  clk.Spi5ClockSelection = clk_source;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  board->microsd_spi->Init.BaudRatePrescaler = prescaler;
  if (HAL_SPI_Init(board->microsd_spi) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }

  return APP_STATUS_OK;
}

/* Sendet ein Kommando und liefert die R1-Antwort. CS bleibt danach LOW,
 * damit der Aufrufer weitere Antwortbytes (R3/R7, Datenbloecke) lesen kann;
 * sd_release() beendet die Transaktion. */
static uint8_t sd_command(const board_interfaces_t *board, uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1 = 0xFFu;

  sd_cs(board, GPIO_PIN_RESET);
  (void)sd_xfer(board, 0xFFu);

  (void)sd_xfer(board, (uint8_t)(0x40u | cmd));
  (void)sd_xfer(board, (uint8_t)(arg >> 24));
  (void)sd_xfer(board, (uint8_t)(arg >> 16));
  (void)sd_xfer(board, (uint8_t)(arg >> 8));
  (void)sd_xfer(board, (uint8_t)arg);
  (void)sd_xfer(board, crc);

  /* Karte antwortet nach 0..8 Idle-Bytes; R1 hat Bit 7 = 0. */
  for (int i = 0; i < 10; ++i)
  {
    r1 = sd_xfer(board, 0xFFu);
    if ((r1 & 0x80u) == 0u)
    {
      break;
    }
  }

  return r1;
}

/* Beendet eine Kommandosequenz: CS anheben und der Karte 8 Takte geben,
 * damit sie die Datenleitung wieder freigibt. */
static void sd_release(const board_interfaces_t *board)
{
  sd_cs(board, GPIO_PIN_SET);
  (void)sd_xfer(board, 0xFFu);
}

/* Liest einen einzelnen 512-Byte-Block (CMD17) zur Verifikation des Datenpfads. */
static app_status_t sd_read_block(const board_interfaces_t *board, uint32_t addr, uint8_t *buf)
{
  uint8_t r1 = sd_command(board, SD_CMD17_READ_SINGLE, addr, 0xFFu);

  if (r1 != 0x00u)
  {
    sd_release(board);
    return APP_STATUS_ERROR;
  }

  /* Auf das Datentoken 0xFE warten. */
  uint32_t start = HAL_GetTick();
  uint8_t token = 0xFFu;
  while ((token = sd_xfer(board, 0xFFu)) == 0xFFu)
  {
    if ((HAL_GetTick() - start) > SD_TOKEN_TIMEOUT_MS)
    {
      sd_release(board);
      return APP_STATUS_ERROR;
    }
  }
  if (token != SD_DATA_TOKEN)
  {
    sd_release(board);
    return APP_STATUS_ERROR;
  }

  for (uint32_t i = 0; i < SD_BLOCK_SIZE; ++i)
  {
    buf[i] = sd_xfer(board, 0xFFu);
  }
  (void)sd_xfer(board, 0xFFu); /* CRC verwerfen */
  (void)sd_xfer(board, 0xFFu);

  sd_release(board);
  return APP_STATUS_OK;
}

app_status_t storage_logger_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  sd_card_type_t card_type = SD_CARD_NONE;
  uint8_t r1;

  storage_logger_ready = false;

  /* CSI-Oszillator (4 MHz) einschalten; er dient nur als langsame
   * SPI5-Taktquelle waehrend der Karteninitialisierung. */
  RCC_OscInitTypeDef osc = { 0 };
  osc.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  osc.CSIState = RCC_CSI_ON;
  osc.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    printf("[microSD] CSI-Oszillator liess sich nicht aktivieren.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Init-Phase: 4 MHz (CSI) / 16 = 250 kHz. */
  if (sd_spi_config(board, RCC_SPI5CLKSOURCE_CSI, SPI_BAUDRATEPRESCALER_16) != APP_STATUS_OK)
  {
    printf("[microSD] SPI5-Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Mindestens 74 Takte mit CS high, damit die Karte in den SPI-Modus wechseln kann. */
  sd_cs(board, GPIO_PIN_SET);
  for (int i = 0; i < 10; ++i)
  {
    (void)sd_xfer(board, 0xFFu);
  }

  /* CMD0: Karte in den Idle-Zustand (SPI-Modus) versetzen. */
  r1 = 0xFFu;
  for (int attempt = 0; attempt < 5 && r1 != SD_R1_IDLE; ++attempt)
  {
    r1 = sd_command(board, SD_CMD0_GO_IDLE, 0u, 0x95u);
    sd_release(board);
  }
  if (r1 != SD_R1_IDLE)
  {
    printf("[microSD] keine Karte gefunden (CMD0-Antwort 0x%02X).\r\n", r1);
    return APP_STATUS_ERROR;
  }

  /* CMD8: unterscheidet SD-Version 2 (antwortet mit Echo) von Version 1. */
  r1 = sd_command(board, SD_CMD8_SEND_IF_COND, 0x000001AAu, 0x87u);
  if (r1 == SD_R1_IDLE)
  {
    uint8_t r7[4];
    for (int i = 0; i < 4; ++i)
    {
      r7[i] = sd_xfer(board, 0xFFu);
    }
    sd_release(board);
    if (r7[2] != 0x01u || r7[3] != 0xAAu)
    {
      printf("[microSD] CMD8-Echo ungueltig (%02X %02X %02X %02X).\r\n", r7[0], r7[1], r7[2], r7[3]);
      return APP_STATUS_ERROR;
    }
    card_type = SD_CARD_V2_SC;
  }
  else
  {
    sd_release(board);
    if ((r1 & SD_R1_ILLEGAL_CMD) == 0u)
    {
      printf("[microSD] CMD8 fehlgeschlagen (0x%02X).\r\n", r1);
      return APP_STATUS_ERROR;
    }
    card_type = SD_CARD_V1;
  }

  /* ACMD41 wiederholen, bis die Karte die Initialisierung abgeschlossen hat. */
  uint32_t start = HAL_GetTick();
  uint32_t acmd41_arg = (card_type == SD_CARD_V1) ? 0u : 0x40000000u; /* HCS-Bit */
  do
  {
    r1 = sd_command(board, SD_CMD55_APP_CMD, 0u, 0xFFu);
    sd_release(board);
    if (r1 > SD_R1_IDLE)
    {
      break;
    }
    r1 = sd_command(board, SD_ACMD41_SD_SEND_OP, acmd41_arg, 0xFFu);
    sd_release(board);
    if ((HAL_GetTick() - start) > SD_ACMD41_TIMEOUT_MS)
    {
      break;
    }
  } while (r1 != 0x00u);
  if (r1 != 0x00u)
  {
    printf("[microSD] Initialisierung nicht abgeschlossen (ACMD41-Antwort 0x%02X).\r\n", r1);
    return APP_STATUS_ERROR;
  }

  /* CMD58: OCR lesen, CCS-Bit unterscheidet SDHC (Blockadressierung) von SDSC. */
  if (card_type != SD_CARD_V1)
  {
    r1 = sd_command(board, SD_CMD58_READ_OCR, 0u, 0xFFu);
    if (r1 == 0x00u)
    {
      uint8_t ocr[4];
      for (int i = 0; i < 4; ++i)
      {
        ocr[i] = sd_xfer(board, 0xFFu);
      }
      if ((ocr[0] & 0x40u) != 0u)
      {
        card_type = SD_CARD_V2_HC;
      }
    }
    sd_release(board);
  }

  /* Datenbetrieb: PCLK3 (250 MHz) / 64 = 3,9 MHz. */
  if (sd_spi_config(board, RCC_SPI5CLKSOURCE_PCLK3, SPI_BAUDRATEPRESCALER_64) != APP_STATUS_OK)
  {
    printf("[microSD] Umschalten auf Datentakt fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  const char *type_name = (card_type == SD_CARD_V2_HC) ? "SDHC/SDXC"
                        : (card_type == SD_CARD_V2_SC) ? "SDSC v2"
                                                       : "SD v1";
  printf("[microSD] Karte initialisiert (%s).\r\n", type_name);

  /* Lesetest: Sektor 0 (Boot-Sektor/MBR endet mit der Signatur 0x55 0xAA). */
  static uint8_t block[SD_BLOCK_SIZE];
  if (sd_read_block(board, 0u, block) != APP_STATUS_OK)
  {
    printf("[microSD] Lesetest von Sektor 0 fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }
  printf("[microSD] Sektor 0 gelesen, Signatur %02X %02X (erwartet 55 AA).\r\n",
         block[510], block[511]);

  storage_logger_ready = true;
  return APP_STATUS_OK;
}

app_status_t storage_logger_write_sample(const app_sample_t *sample)
{
  /* Spaeter: app_sample_t als CSV- oder Binaerdatensatz auf die microSD schreiben. */
  (void)sample;
  return APP_STATUS_NOT_READY;
}

bool storage_logger_is_ready(void)
{
  return storage_logger_ready;
}
