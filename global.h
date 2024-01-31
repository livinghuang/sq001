#ifndef GLOBAL_H
#define GLOBAL_H

#include "Arduino.h"
#include "esp_system.h"
#include "LoRaWan_APP.h"
#include "bsp.h"
#include "WiFi.h"
#include "Wire.h"
#include "esp_sleep.h"
#include <esp_timer.h>
#include <mbedtls/sha256.h>

#include "bat.h"
#include "lorawan.h"
#include "lora.h"
#include "Dps310.h"
#include "DpsClass.h"
#include "HDC1080.h"
#include "BMP280.h"
#include "led.h"
#include "_wifi.h"
#include "sensor.h"
#include "rs485.h"

typedef enum
{
  WIFI_CONNECT_TEST_INIT,
  WIFI_CONNECT_TEST,
  WIFI_SCAN_TEST,
  LORAWAN_TEST_INIT,
  LORAWAN_COMMUNICATION_TEST,
  LORA_TEST_INIT,
  LORA_COMMUNICATION_TEST,
  HDC1080_TEST_INIT,
  HDC1080_TEST,
  BMP280_TEST_INIT,
  BMP280_TEST,
  DSP310_TEST_INIT,
  DSP310_TEST,
  DEEPSLEEP_BY_TIMER_TEST,
  DEEPSLEEP_BY_GPIO_TEST,
  LED_INIT,
  LED_TEST,
  BAT_TEST,
  RS485_INIT,
  RS485_TEST,
  _TEST_INIT,
  _TEST,
} test_status_t;

extern bool resendflag;
extern bool deepsleepflag;
extern bool interrupt_flag;
extern bool reset_run_with_time_escape;

void printHex(byte *data, int length);
void run_with_time_escape(uint64_t escape_period_ms, void (*callback)(), void (*stop_callback)());
esp_sleep_wakeup_cause_t print_wakeup_reason();
uint64_t get_chip_id();
#endif