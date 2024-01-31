#include "global.h"
void power_On_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, HIGH);
  digitalWrite(pSCL, HIGH);
  delay(5);
  digitalWrite(pVext, HIGH);
}

void power_Off_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, LOW);
  digitalWrite(pSCL, LOW);
  digitalWrite(pVext, HIGH);
}

void enter_deepsleep(void)
{
  Radio.Sleep();
  SPI.end();
  pinMode(RADIO_DIO_1, ANALOG);
  pinMode(RADIO_NSS, ANALOG);
  pinMode(RADIO_RESET, ANALOG);
  pinMode(RADIO_BUSY, ANALOG);
  pinMode(LORA_CLK, ANALOG);
  pinMode(LORA_MISO, ANALOG);
  pinMode(LORA_MOSI, ANALOG);
  esp_sleep_enable_timer_wakeup(600 * 1000 * (uint64_t)1000);
  esp_deep_sleep_start();
}