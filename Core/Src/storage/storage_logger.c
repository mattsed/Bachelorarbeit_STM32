#include "storage/storage_logger.h"

#include <stdio.h>
#include <string.h>

#include "board/board.h"

#include "ff.h"
#include "diskio.h"

/* ==========================================================================
 * SD-Karten-Low-Level (SPI-Modus, vgl. SD Physical Layer Spec).
 * Verifiziert am 16.07.2026 (Init + Sektor-0-Lesetest), am 17.07.2026 um
 * CMD24-Schreiben erweitert. Dient als diskio-Backend fuer FatFS.
 * ========================================================================== */

#define SD_CMD0_GO_IDLE          0u
#define SD_CMD8_SEND_IF_COND     8u
#define SD_CMD17_READ_SINGLE     17u
#define SD_CMD24_WRITE_SINGLE    24u
#define SD_CMD55_APP_CMD         55u
#define SD_CMD58_READ_OCR        58u
#define SD_ACMD41_SD_SEND_OP     41u

#define SD_R1_IDLE               0x01u
#define SD_R1_ILLEGAL_CMD        0x04u

#define SD_BLOCK_SIZE            512u
#define SD_ACMD41_TIMEOUT_MS     1000u
#define SD_TOKEN_TIMEOUT_MS      200u
#define SD_WRITE_TIMEOUT_MS      500u
#define SD_DATA_TOKEN            0xFEu

typedef enum
{
  SD_CARD_NONE = 0,
  SD_CARD_V1,
  SD_CARD_V2_SC,
  SD_CARD_V2_HC,
} sd_card_type_t;

static sd_card_type_t sd_card_type = SD_CARD_NONE;

static void sd_cs(const board_interfaces_t *board, GPIO_PinState state)
{
  HAL_GPIO_WritePin(board->microsd_cs.port, board->microsd_cs.pin, state);
}

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

/* Sendet ein Kommando und liefert die R1-Antwort. CS bleibt danach LOW. */
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

static void sd_release(const board_interfaces_t *board)
{
  sd_cs(board, GPIO_PIN_SET);
  (void)sd_xfer(board, 0xFFu);
}

/* Wandelt eine Sektornummer in das Adressformat der Karte um:
 * SDHC/SDXC adressiert in Bloecken, aeltere Karten in Bytes. */
static uint32_t sd_sector_to_addr(uint32_t sector)
{
  return (sd_card_type == SD_CARD_V2_HC) ? sector : sector * SD_BLOCK_SIZE;
}

/* Liest einen einzelnen 512-Byte-Block (CMD17). */
static app_status_t sd_read_block(const board_interfaces_t *board, uint32_t sector, uint8_t *buf)
{
  uint8_t r1 = sd_command(board, SD_CMD17_READ_SINGLE, sd_sector_to_addr(sector), 0xFFu);

  if (r1 != 0x00u)
  {
    sd_release(board);
    return APP_STATUS_ERROR;
  }

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

/* Schreibt einen einzelnen 512-Byte-Block (CMD24) und wartet das interne
 * Programmieren der Karte ab (Busy = Karte haelt die Datenleitung low). */
static app_status_t sd_write_block(const board_interfaces_t *board, uint32_t sector, const uint8_t *buf)
{
  uint8_t r1 = sd_command(board, SD_CMD24_WRITE_SINGLE, sd_sector_to_addr(sector), 0xFFu);

  if (r1 != 0x00u)
  {
    sd_release(board);
    return APP_STATUS_ERROR;
  }

  (void)sd_xfer(board, 0xFFu);          /* Luecke vor dem Datenpaket */
  (void)sd_xfer(board, SD_DATA_TOKEN);  /* Starttoken */
  for (uint32_t i = 0; i < SD_BLOCK_SIZE; ++i)
  {
    (void)sd_xfer(board, buf[i]);
  }
  (void)sd_xfer(board, 0xFFu);          /* Dummy-CRC */
  (void)sd_xfer(board, 0xFFu);

  uint8_t resp = sd_xfer(board, 0xFFu); /* Data Response Token */
  if ((resp & 0x1Fu) != 0x05u)          /* 0x05 = Daten angenommen */
  {
    sd_release(board);
    return APP_STATUS_ERROR;
  }

  uint32_t start = HAL_GetTick();
  while (sd_xfer(board, 0xFFu) != 0xFFu)
  {
    if ((HAL_GetTick() - start) > SD_WRITE_TIMEOUT_MS)
    {
      sd_release(board);
      return APP_STATUS_ERROR;
    }
  }

  sd_release(board);
  return APP_STATUS_OK;
}

/* Karteninitialisierung nach SD-Spezifikation (CMD0/CMD8/ACMD41/CMD58). */
static app_status_t sd_hw_init(const board_interfaces_t *board)
{
  uint8_t r1;

  sd_card_type = SD_CARD_NONE;

  if (sd_spi_config(board, RCC_SPI5CLKSOURCE_CSI, SPI_BAUDRATEPRESCALER_16) != APP_STATUS_OK)
  {
    printf("[microSD] SPI5-Konfiguration fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  /* Mindestens 74 Takte mit CS high fuer den Wechsel in den SPI-Modus. */
  sd_cs(board, GPIO_PIN_SET);
  for (int i = 0; i < 10; ++i)
  {
    (void)sd_xfer(board, 0xFFu);
  }

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

  sd_card_type_t type;
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
      printf("[microSD] CMD8-Echo ungueltig.\r\n");
      return APP_STATUS_ERROR;
    }
    type = SD_CARD_V2_SC;
  }
  else
  {
    sd_release(board);
    if ((r1 & SD_R1_ILLEGAL_CMD) == 0u)
    {
      printf("[microSD] CMD8 fehlgeschlagen (0x%02X).\r\n", r1);
      return APP_STATUS_ERROR;
    }
    type = SD_CARD_V1;
  }

  uint32_t start = HAL_GetTick();
  uint32_t acmd41_arg = (type == SD_CARD_V1) ? 0u : 0x40000000u;
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
    printf("[microSD] Initialisierung nicht abgeschlossen (0x%02X).\r\n", r1);
    return APP_STATUS_ERROR;
  }

  if (type != SD_CARD_V1)
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
        type = SD_CARD_V2_HC;
      }
    }
    sd_release(board);
  }

  if (sd_spi_config(board, RCC_SPI5CLKSOURCE_PCLK3, SPI_BAUDRATEPRESCALER_64) != APP_STATUS_OK)
  {
    printf("[microSD] Umschalten auf Datentakt fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  sd_card_type = type;
  printf("[microSD] Karte initialisiert (%s).\r\n",
         (type == SD_CARD_V2_HC) ? "SDHC/SDXC" : (type == SD_CARD_V2_SC) ? "SDSC v2" : "SD v1");
  return APP_STATUS_OK;
}

/* ==========================================================================
 * diskio-Backend fuer FatFS (ein Laufwerk, Laufwerksnummer 0).
 * ========================================================================== */

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != 0u || sd_card_type == SD_CARD_NONE)
  {
    return STA_NOINIT;
  }
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != 0u)
  {
    return STA_NOINIT;
  }
  if (sd_card_type == SD_CARD_NONE)
  {
    if (sd_hw_init(board_get_interfaces()) != APP_STATUS_OK)
    {
      return STA_NOINIT;
    }
  }
  return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  const board_interfaces_t *board = board_get_interfaces();

  if (pdrv != 0u || sd_card_type == SD_CARD_NONE)
  {
    return RES_NOTRDY;
  }
  for (UINT i = 0; i < count; ++i)
  {
    if (sd_read_block(board, (uint32_t)sector + i, &buff[i * SD_BLOCK_SIZE]) != APP_STATUS_OK)
    {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  const board_interfaces_t *board = board_get_interfaces();

  if (pdrv != 0u || sd_card_type == SD_CARD_NONE)
  {
    return RES_NOTRDY;
  }
  for (UINT i = 0; i < count; ++i)
  {
    if (sd_write_block(board, (uint32_t)sector + i, &buff[i * SD_BLOCK_SIZE]) != APP_STATUS_OK)
    {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  (void)buff;

  if (pdrv != 0u)
  {
    return RES_PARERR;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      /* sd_write_block wartet das Busy-Ende bereits ab. */
      return RES_OK;
    case GET_SECTOR_SIZE:
      *(WORD *)buff = SD_BLOCK_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

/* ==========================================================================
 * Storage-Logger (FatFS-Ebene).
 * ========================================================================== */

static FATFS storage_fs;
static FIL storage_log_file;
static bool storage_logger_ready = false;
static uint32_t storage_sample_count = 0;

/* Alle N Datensaetze auf die Karte synchronisieren: begrenzt den Datenverlust
 * bei Stromausfall (Sturz, Akku ab) auf rund eine halbe Sekunde. */
#define STORAGE_SYNC_EVERY_N_SAMPLES 25u

app_status_t storage_logger_init(void)
{
  static FIL fil;
  char line[64];
  FRESULT res;

  storage_logger_ready = false;

  res = f_mount(&storage_fs, "", 1);
  if (res != FR_OK)
  {
    printf("[microSD] FatFS-Mount fehlgeschlagen (FRESULT=%d).\r\n", (int)res);
    return APP_STATUS_ERROR;
  }

  /* Schreib-/Lesetest: Datei anlegen, Zeile schreiben, zuruecklesen. */
  res = f_open(&fil, "BRINGUP.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK)
  {
    printf("[microSD] Testdatei liess sich nicht anlegen (FRESULT=%d).\r\n", (int)res);
    return APP_STATUS_ERROR;
  }
  f_printf(&fil, "Datenlogger-Bring-up ok, Tick=%lu ms\n", (unsigned long)HAL_GetTick());
  res = f_close(&fil);
  if (res != FR_OK)
  {
    printf("[microSD] Testdatei liess sich nicht schreiben (FRESULT=%d).\r\n", (int)res);
    return APP_STATUS_ERROR;
  }

  res = f_open(&fil, "BRINGUP.TXT", FA_READ);
  if (res != FR_OK || f_gets(line, sizeof(line), &fil) == NULL)
  {
    printf("[microSD] Testdatei liess sich nicht zuruecklesen (FRESULT=%d).\r\n", (int)res);
    (void)f_close(&fil);
    return APP_STATUS_ERROR;
  }
  (void)f_close(&fil);

  /* Zeilenumbruch fuer die Ausgabe entfernen. */
  line[strcspn(line, "\r\n")] = '\0';
  printf("[microSD] FatFS ok, BRINGUP.TXT: \"%s\"\r\n", line);

  return storage_logger_start();
}

app_status_t storage_logger_start(void)
{
  char name[16];
  FILINFO info;
  FRESULT res;

  if (storage_logger_ready)
  {
    return APP_STATUS_OK; /* laeuft bereits */
  }

  /* Logdatei mit fortlaufender Nummer anlegen (nichts ueberschreiben). */
  unsigned int num = 0;
  for (; num < 1000u; ++num)
  {
    snprintf(name, sizeof(name), "LOG_%03u.CSV", num);
    if (f_stat(name, &info) == FR_NO_FILE)
    {
      break;
    }
  }
  res = f_open(&storage_log_file, name, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK)
  {
    printf("[microSD] Logdatei %s liess sich nicht anlegen (FRESULT=%d).\r\n", name, (int)res);
    return APP_STATUS_ERROR;
  }
  f_puts("t_ms;fix;lat_e7;lon_e7;v_mm_s;utc_ms;"
         "p_vorne_raw;p_hinten_raw;"
         "imu_ax;imu_ay;imu_az;imu_gx;imu_gy;imu_gz;"
         "acc400_x;acc400_y;acc400_z\n", &storage_log_file);
  if (f_sync(&storage_log_file) != FR_OK)
  {
    printf("[microSD] Logdatei-Kopfzeile liess sich nicht schreiben.\r\n");
    return APP_STATUS_ERROR;
  }
  printf("[microSD] Logdatei %s angelegt, Aufzeichnung laeuft.\r\n", name);

  storage_sample_count = 0;
  storage_logger_ready = true;
  return APP_STATUS_OK;
}

app_status_t storage_logger_stop(void)
{
  if (!storage_logger_ready)
  {
    return APP_STATUS_OK; /* nichts zu stoppen */
  }

  storage_logger_ready = false;
  if (f_close(&storage_log_file) != FR_OK)
  {
    printf("[microSD] Logdatei liess sich nicht schliessen.\r\n");
    return APP_STATUS_ERROR;
  }
  printf("[microSD] Aufzeichnung gestoppt, Datei geschlossen (%lu Datensaetze).\r\n",
         (unsigned long)storage_sample_count);
  return APP_STATUS_OK;
}

app_status_t storage_logger_write_sample(const app_sample_t *sample)
{
  if (!storage_logger_ready || sample == NULL)
  {
    return APP_STATUS_NOT_READY;
  }

  int written = f_printf(&storage_log_file,
                         "%lu;%d;%ld;%ld;%lu;%lu;%u;%u;%d;%d;%d;%d;%d;%d;%d;%d;%d\n",
                         (unsigned long)sample->timestamp_ms,
                         sample->gnss.fix_valid ? 1 : 0,
                         (long)sample->gnss.latitude_e7,
                         (long)sample->gnss.longitude_e7,
                         (unsigned long)sample->gnss.speed_mm_s,
                         (unsigned long)sample->gnss.utc_time_ms,
                         sample->brake_pressure.front_raw,
                         sample->brake_pressure.back_raw,
                         sample->imu.accel_x_raw, sample->imu.accel_y_raw, sample->imu.accel_z_raw,
                         sample->imu.gyro_x_raw, sample->imu.gyro_y_raw, sample->imu.gyro_z_raw,
                         sample->acc_400g.accel_x_raw, sample->acc_400g.accel_y_raw,
                         sample->acc_400g.accel_z_raw);
  if (written < 0)
  {
    return APP_STATUS_ERROR;
  }

  if (++storage_sample_count % STORAGE_SYNC_EVERY_N_SAMPLES == 0u)
  {
    if (f_sync(&storage_log_file) != FR_OK)
    {
      return APP_STATUS_ERROR;
    }
  }
  return APP_STATUS_OK;
}

bool storage_logger_is_ready(void)
{
  return storage_logger_ready;
}
