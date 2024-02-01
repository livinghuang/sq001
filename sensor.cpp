#include "global.h"

struct hdc1080_data
{
  float temperature;
  float humidity;
} hdc1080_result;

HDC1080 hdc1080;

bool hdc1080_fetch(void)
{
  if (!hdc1080.begin())
  {
    return 0;
  }
  float temp = hdc1080.readTemperature();
  float Humidity = hdc1080.readHumidity();
  temp = hdc1080.readTemperature();
  Humidity = hdc1080.readHumidity();
  hdc1080.end();
  Serial.printf("T=%.2f degC, Humidity=%.2f %\n", temp, Humidity);
  hdc1080_result.temperature = temp;
  hdc1080_result.humidity = Humidity;
  return 1;
}

void fetchSensorData()
{
  power_On_Sensor_Bus();
  delay(5);
  uint8_t fetch_times = 3;
  while (fetch_times--)
  {
    hdc1080_fetch();
    delay(1000);
  }
  power_Off_Sensor_Bus();
}