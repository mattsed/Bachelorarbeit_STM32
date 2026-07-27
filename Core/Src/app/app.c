#include "app/app.h"

#include <stdio.h>
#include <string.h>

#include "board/board.h"
#include "comms/ble_bluenrg.h"
#include "sensors/acc_adxl373.h"
#include "sensors/brake_pressure.h"
#include "sensors/gnss.h"
#include "sensors/imu_lsm6dso.h"
#include "storage/storage_logger.h"

/* Messtakt: alle 20 ms ein kompletter Datensatz (50 Hz). GNSS aktualisiert
 * sich selbst nur mit 1 Hz; jede Zeile traegt den letzten bekannten Stand. */
#define APP_SAMPLE_PERIOD_MS  20u
#define APP_LED_PERIOD_MS     500u
#define APP_BUTTON_DEBOUNCE_MS 50u

/* SD-Fehlerbehandlung: schnelles LED-Blinken signalisiert den Fehler am
 * Lenker, alle 2 s ein Wiederherstellungsversuch (Karte neu initialisieren,
 * neue Logdatei). Haeufiger lohnt nicht und wuerde die Messschleife bremsen. */
#define APP_LED_ERROR_PERIOD_MS   100u
#define APP_SD_RECOVER_PERIOD_MS  2000u

/* true, solange ein SD-Schreibfehler ansteht und die Wiederherstellung
 * noch nicht geglueckt ist. */
static bool app_sd_error = false;

/* --------------------------------------------------------------------------
 * Unabhaengiger Watchdog (IWDG): eigener Zaehler auf dem internen LSI-
 * Oszillator (~32 kHz), der den MCU zuruecksetzt, wenn er nicht regelmaessig
 * "gefuettert" wird -- schuetzt im Feld gegen haengende Firmware (z. B.
 * blockierter SPI-Bus nach Wackelkontakt). Direkte Registerzugriffe statt
 * HAL, weil das IWDG-HAL-Modul im CubeMX-Projekt deaktiviert ist und eine
 * Aktivierung in stm32h5xx_hal_conf.h bei der naechsten Code-Generierung
 * verloren ginge. Einmal gestartet laesst sich das IWDG nicht mehr stoppen.
 * 32 kHz / 256 (Vorteiler) = 125 Hz Zaehltakt; Reload 500 => ~4 s Timeout.
 * -------------------------------------------------------------------------- */
#define APP_IWDG_KEY_ENABLE     0x0000CCCCu  /* Schluesselwerte laut Referenzhandbuch */
#define APP_IWDG_KEY_ACCESS     0x00005555u
#define APP_IWDG_KEY_REFRESH    0x0000AAAAu
#define APP_IWDG_PRESCALER_256  6u
#define APP_IWDG_RELOAD_4S      500u

static void app_watchdog_start(void)
{
  /* Watchdog anhalten, solange der Debugger die CPU stoppt -- sonst wuerde
   * jeder Breakpoint nach 4 s einen Reset ausloesen. */
  __HAL_DBGMCU_FREEZE_IWDG();

  IWDG->KR  = APP_IWDG_KEY_ENABLE;  /* startet Zaehler, LSI laeuft automatisch an */
  IWDG->KR  = APP_IWDG_KEY_ACCESS;  /* PR/RLR zum Beschreiben freischalten */
  IWDG->PR  = APP_IWDG_PRESCALER_256;
  IWDG->RLR = APP_IWDG_RELOAD_4S;

  /* Die Register liegen in der langsamen LSI-Taktdomaene; SR zeigt an,
   * solange die Uebernahme laeuft. Danach den Zaehler frisch laden. */
  uint32_t start = HAL_GetTick();
  while (IWDG->SR != 0u)
  {
    if ((HAL_GetTick() - start) > 50u)
    {
      break;
    }
  }
  IWDG->KR = APP_IWDG_KEY_REFRESH;
  printf("[App] Watchdog aktiv (Reset nach ~4 s ohne Lebenszeichen).\r\n");
}

/* B1-USER-Knopf (PC13): stoppt bzw. startet die Aufzeichnung. Ausgewertet
 * wird der entprellte Wechsel auf "gedrueckt" (Pegel high). */
static void app_check_user_button(void)
{
  static GPIO_PinState stable_state = GPIO_PIN_RESET;
  static GPIO_PinState last_raw = GPIO_PIN_RESET;
  static uint32_t last_change_ms = 0;

  GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  uint32_t now = HAL_GetTick();

  /* Schritt 1: Bei jeder Pegelaenderung nur den Zeitpunkt merken --
   * das mechanische Prellen des Kontakts erzeugt hier viele schnelle
   * Wechsel, die alle wieder von vorn zaehlen. */
  if (raw != last_raw)
  {
    last_raw = raw;
    last_change_ms = now;
    return;
  }
  /* Schritt 2: Erst wenn der Pegel 50 ms lang stabil war UND sich vom
   * zuletzt akzeptierten Zustand unterscheidet, gilt er als echt. */
  if ((now - last_change_ms) < APP_BUTTON_DEBOUNCE_MS || raw == stable_state)
  {
    return;
  }
  stable_state = raw;

  /* Schritt 3: Nur die Flanke "losgelassen -> gedrueckt" schaltet um
   * (das Loslassen selbst loest nichts aus). */
  if (stable_state == GPIO_PIN_SET) /* Knopf gedrueckt */
  {
    if (storage_logger_is_ready() || app_sd_error)
    {
      /* Stopp gewinnt auch im Fehlerfall: Fehlerzustand verwerfen, damit
       * die automatische Wiederherstellung das Logging nicht gegen den
       * Willen des Nutzers wieder anwirft. */
      app_sd_error = false;
      (void)storage_logger_stop();
    }
    else
    {
      (void)storage_logger_start();
    }
  }
}

app_status_t app_init(void)
{
  /* Falls der letzte Neustart vom Watchdog kam, das deutlich melden --
   * im Feld der wichtigste Hinweis darauf, dass die Firmware gehangen hat. */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  {
    printf("[App] ACHTUNG: Neustart wurde vom Watchdog ausgeloest (Firmware hing).\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Alle Module einmalig initialisieren. Die Rueckgabewerte werden bewusst
   * verworfen ((void)-Cast): Ein fehlgeschlagenes Modul (z. B. keine
   * SD-Karte) soll den Start der uebrigen nicht verhindern. Jedes Modul
   * meldet seinen Status selbst per printf auf der Konsole. */
  (void)board_init();
  (void)gnss_init();
  (void)brake_pressure_init();
  (void)imu_lsm6dso_init();
  (void)acc_adxl373_init();
  (void)storage_logger_init();
  (void)ble_bluenrg_init();

  if (storage_logger_is_ready())
  {
    printf("[App] Messzyklus laeuft (%u Hz).\r\n", (unsigned int)(1000u / APP_SAMPLE_PERIOD_MS));
  }
  else
  {
    printf("[App] Kein Logging moeglich (microSD nicht bereit) -- nur Sensor-Poll.\r\n");
  }

  /* Watchdog bewusst erst NACH allen Modul-Inits scharf schalten:
   * gnss_init() darf bis zu 6 s blockieren -- laenger als der 4-s-Timeout. */
  app_watchdog_start();

  return APP_STATUS_OK;
}

void app_run(void)
{
  static uint32_t next_sample_ms = 0;
  static uint32_t next_led_ms = 0;

  /* Watchdog fuettern: einmal pro Schleifendurchlauf (auch im gestoppten
   * Zustand -- die Schleife laeuft dort weiter). Bleibt dieser Aufruf ~4 s
   * aus, haengt die Firmware und der Watchdog resettet den MCU. */
  IWDG->KR = APP_IWDG_KEY_REFRESH;

  /* Hintergrundaufgaben: NMEA-Strom leeren, BLE-Ereignisse abholen,
   * Start/Stopp-Knopf auswerten. */
  (void)gnss_poll();
  (void)ble_bluenrg_poll();
  app_check_user_button();

  uint32_t now = HAL_GetTick();

  /* Nach einem SD-Fehler: ratenbegrenzt versuchen, Karte und Logging
   * wiederherzustellen (z. B. nachdem die Karte neu gesteckt wurde). */
  static uint32_t next_recover_ms = 0;
  if (app_sd_error && now >= next_recover_ms)
  {
    next_recover_ms = now + APP_SD_RECOVER_PERIOD_MS;
    if (storage_logger_recover() == APP_STATUS_OK)
    {
      app_sd_error = false;
      printf("[App] SD wiederhergestellt -- Logging laeuft in neuer Datei weiter.\r\n");
    }
  }

  /* Gruene LED (PB0): blinkt langsam = Aufzeichnung laeuft, dauerhaft an =
   * gestoppt, blinkt schnell = SD-Fehler (am Lenker sofort erkennbar). */
  if (app_sd_error)
  {
    if (now >= next_led_ms)
    {
      next_led_ms = now + APP_LED_ERROR_PERIOD_MS;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }
  }
  else if (storage_logger_is_ready())
  {
    if (now >= next_led_ms)
    {
      next_led_ms = now + APP_LED_PERIOD_MS;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  }

  /* Messtakt: erst wenn die naechste 20-ms-Marke erreicht ist, geht es
   * weiter -- sonst endet die Schleifenrunde hier. Der naechste Termin wird
   * relativ zum vorigen fortgeschrieben (nicht relativ zu "jetzt"), damit
   * sich kleine Verspaetungen nicht aufsummieren. */
  if (now < next_sample_ms)
  {
    return;
  }
  next_sample_ms = (next_sample_ms == 0u) ? now + APP_SAMPLE_PERIOD_MS
                                          : next_sample_ms + APP_SAMPLE_PERIOD_MS;

  /* Einen kompletten Messdatensatz einsammeln: ein Zeitstempel, dann alle
   * Sensoren unmittelbar nacheinander -- so gehoeren die Werte einer
   * CSV-Zeile zeitlich auf <1 ms zusammen. Nicht bereite Module lassen
   * ihre Felder einfach auf 0 (memset oben). */
  app_sample_t sample;
  memset(&sample, 0, sizeof(sample));
  sample.timestamp_ms = now;
  (void)gnss_read(&sample.gnss);
  (void)brake_pressure_read(&sample.brake_pressure);
  (void)imu_lsm6dso_read(&sample.imu);
  (void)acc_adxl373_read(&sample.acc_400g);

  /* Datensatz als CSV-Zeile auf die microSD schreiben (falls Logging aktiv).
   * Ein Schreibfehler (Karte gezogen, Karte defekt) setzt den Fehlerzustand:
   * LED blinkt schnell, oben laeuft alle 2 s die Wiederherstellung. Waehrend
   * des Fehlers wird nicht weiter geschrieben -- diese Daten gehen verloren,
   * die Sensoren laufen aber weiter. APP_STATUS_NOT_READY (Logging gestoppt)
   * ist kein Fehler. */
  if (!app_sd_error &&
      storage_logger_write_sample(&sample) == APP_STATUS_ERROR)
  {
    app_sd_error = true;
    printf("[App] SD-Schreibfehler -- Aufzeichnung unterbrochen, versuche Wiederherstellung.\r\n");
  }
}
