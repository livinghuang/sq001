
#include "global.h"

float getBatteryVoltage()
{
  uint32_t sum = 0;
  uint32_t test_min = 695;
  uint32_t test_max = 1030;
  for (size_t i = 0; i < 16; i++)
  {
    sum += analogRead(2);
    delay(10);
  }
  float avg = (float)(sum >> 4) / 4095 * 2500;
  Serial.print("avg");
  Serial.println(avg);
  return ((avg - test_min) * (4.2 - 3) / (test_max - test_min) + 3);
}

uint8_t getBatteryLevel()
{
  const float maxBattery = 4.2;
  const float minBattery = 3.0;
  const float batVolt = getBatteryVoltage();
  const float batVoltage = fmax(minBattery, fmin(maxBattery, batVolt));
  uint8_t batLevel = BAT_LEVEL_EMPTY + ((batVoltage - minBattery) / (maxBattery - minBattery)) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
  if (batVolt > 4.2)
  {
    batLevel = 255;
  }
  if (batVolt < 3.0)
  {
    batLevel = 0;
  }
  Serial.print("{");
  Serial.println(batVoltage);
  Serial.print(batLevel);
  Serial.println("}");
  return batLevel;
}