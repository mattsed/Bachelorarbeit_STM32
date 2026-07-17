#include "sensors/brake_pressure.h"

#include <stdio.h>

#include "board/board.h"

/* Bosch PSS-140 (ratiometrisch, US = 5 V nominal): Offset 0,5 V,
 * Empfindlichkeit 28,57 mV/bar. Vor dem ADC sitzt pro Kanal ein
 * Spannungsteiler R1 = 15 kOhm / R2 = 33 kOhm (Faktor 33/48 = 0,6875),
 * damit die maximal 4,725 V des Sensors unter der 3,3-V-Referenz bleiben.
 * Bei anderem Teiler nur BRAKE_DIVIDER_RATIO anpassen. */
#define BRAKE_VREF_VOLT       3.3f
#define BRAKE_ADC_MAX         4095.0f
#define BRAKE_DIVIDER_RATIO   (33.0f / 48.0f)
#define BRAKE_SENSOR_OFFSET_V 0.5f
#define BRAKE_SENSOR_V_PER_BAR 0.028571f
#define BRAKE_AVG_SAMPLES     8u

/* Wird true, sobald die ADC-Kanaele fuer die Bremsdrucksensoren bereit sind. */
static bool brake_pressure_ready = false;

/* Rechnet einen ADC-Rohwert in die Sensor-Ausgangsspannung (vor dem Teiler)
 * und weiter in bar um. Werte unter dem 0,5-V-Offset ergeben 0 bar. */
static float brake_raw_to_bar(uint16_t raw, float *sensor_volt)
{
  float pin_volt = ((float)raw / BRAKE_ADC_MAX) * BRAKE_VREF_VOLT;
  float u_sensor = pin_volt / BRAKE_DIVIDER_RATIO;

  if (sensor_volt != NULL)
  {
    *sensor_volt = u_sensor;
  }
  if (u_sensor <= BRAKE_SENSOR_OFFSET_V)
  {
    return 0.0f;
  }
  return (u_sensor - BRAKE_SENSOR_OFFSET_V) / BRAKE_SENSOR_V_PER_BAR;
}

/* Fuehrt einen Scan ueber beide Kanaele aus (Rank 1 = PA0/vorne,
 * Rank 2 = PA3/hinten). */
static app_status_t brake_adc_scan(ADC_HandleTypeDef *adc, uint16_t *front, uint16_t *back)
{
  if (HAL_ADC_Start(adc) != HAL_OK)
  {
    return APP_STATUS_ERROR;
  }
  if (HAL_ADC_PollForConversion(adc, 10) != HAL_OK)
  {
    (void)HAL_ADC_Stop(adc);
    return APP_STATUS_ERROR;
  }
  *front = (uint16_t)HAL_ADC_GetValue(adc);
  if (HAL_ADC_PollForConversion(adc, 10) != HAL_OK)
  {
    (void)HAL_ADC_Stop(adc);
    return APP_STATUS_ERROR;
  }
  *back = (uint16_t)HAL_ADC_GetValue(adc);
  (void)HAL_ADC_Stop(adc);
  return APP_STATUS_OK;
}

app_status_t brake_pressure_init(void)
{
  const board_interfaces_t *board = board_get_interfaces();
  uint32_t sum_front = 0;
  uint32_t sum_back = 0;

  brake_pressure_ready = false;

  /* Einmalige Selbstkalibrierung des ADC (Offsetabgleich). */
  if (HAL_ADCEx_Calibration_Start(board->brake_adc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    printf("[Bremsdruck] ADC-Kalibrierung fehlgeschlagen.\r\n");
    return APP_STATUS_ERROR;
  }

  for (uint32_t i = 0; i < BRAKE_AVG_SAMPLES; ++i)
  {
    uint16_t front = 0;
    uint16_t back = 0;
    if (brake_adc_scan(board->brake_adc, &front, &back) != APP_STATUS_OK)
    {
      printf("[Bremsdruck] ADC-Wandlung fehlgeschlagen.\r\n");
      return APP_STATUS_ERROR;
    }
    sum_front += front;
    sum_back += back;
  }

  uint16_t front_raw = (uint16_t)(sum_front / BRAKE_AVG_SAMPLES);
  uint16_t back_raw = (uint16_t)(sum_back / BRAKE_AVG_SAMPLES);
  float front_volt = 0.0f;
  float back_volt = 0.0f;
  float front_bar = brake_raw_to_bar(front_raw, &front_volt);
  float back_bar = brake_raw_to_bar(back_raw, &back_volt);

  /* Ganzzahlig ausgeben (mV / Zehntel-bar), da newlib-nano %f nicht kann. */
  printf("[Bremsdruck] vorne: roh=%u  U_Sensor=%d mV  p=%d.%d bar\r\n",
         front_raw, (int)(front_volt * 1000.0f),
         (int)front_bar, (int)(front_bar * 10.0f) % 10);
  printf("[Bremsdruck] hinten: roh=%u  U_Sensor=%d mV  p=%d.%d bar\r\n",
         back_raw, (int)(back_volt * 1000.0f),
         (int)back_bar, (int)(back_bar * 10.0f) % 10);

  brake_pressure_ready = true;
  return APP_STATUS_OK;
}

app_status_t brake_pressure_read(brake_pressure_data_t *data)
{
  const board_interfaces_t *board = board_get_interfaces();

  if (!brake_pressure_ready || data == NULL)
  {
    return APP_STATUS_NOT_READY;
  }

  if (brake_adc_scan(board->brake_adc, &data->front_raw, &data->back_raw) != APP_STATUS_OK)
  {
    return APP_STATUS_ERROR;
  }
  data->front_bar = brake_raw_to_bar(data->front_raw, NULL);
  data->back_bar = brake_raw_to_bar(data->back_raw, NULL);
  return APP_STATUS_OK;
}

bool brake_pressure_is_ready(void)
{
  return brake_pressure_ready;
}
