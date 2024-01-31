#include "global.h"
// Dps310 Opject
bool resendflag = false;
bool deepsleepflag = false;

RTC_DATA_ATTR int bootCount = 0;
uint64_t chipid;
test_status_t test_status;

void setup()
{
  Serial.begin(115200);
  Mcu.begin();

  print_wakeup_reason();
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  resendflag = false;
  deepsleepflag = false;

  get_chip_id();
  test_status = WIFI_CONNECT_TEST_INIT;
}

void loop()
{
  switch (test_status)
  {
  case WIFI_CONNECT_TEST_INIT:
  {
    wifi_connect_init();
    test_status = WIFI_CONNECT_TEST;
  }
  case WIFI_CONNECT_TEST:
  {
    if (wifi_connect_try(1) == true)
    {
      Serial.println("wifi connect OK");
    }
    wifi_connect_try_num--;
    break;
  }
  case WIFI_SCAN_TEST:
  {
    wifi_scan(1);
    break;
  }
  case LORAWAN_TEST_INIT:
  {
    lorawan_init();
    test_status = LORAWAN_COMMUNICATION_TEST;
    delay(1);
    break;
  }
  case LORAWAN_COMMUNICATION_TEST:
  {
    lorawan_process();
    delay(1);
    break;
  }
  case LORA_TEST_INIT:
  {
    Serial.println("LORA_TEST_INIT");
    Serial.flush();
    lora_init();
    test_status = LORAWAN_COMMUNICATION_TEST;
    delay(1);
    break;
  }
  case LORA_COMMUNICATION_TEST:
  {
    Serial.println("LORA_COMMUNICATION_TEST");
    Serial.flush();
    lora_status_handle();
    delay(1);
    break;
  }
  case HDC1080_TEST_INIT:
  {
    power_On_Sensor_Bus();
    delay(5);
    test_status = HDC1080_TEST;
    break;
  }
  case HDC1080_TEST:
  {
    hdc1080_fetch();
    delay(1000);
    break;
  }
  case BMP280_TEST_INIT:
  {
    power_On_Sensor_Bus();
    delay(5);
    test_status = BMP280_TEST;
    break;
  }
  case BMP280_TEST:
  {
    bmp280_fetch();
    delay(1000);
    break;
  }
  case DSP310_TEST_INIT:
  {
    power_On_Sensor_Bus();
    delay(5);
    Wire.setPins(pSDA, pSCL);
    // Call begin to initialize Dps310PressureSensor
    // The parameter 0x76 is the bus address. The default address is 0x77 and does not need to be given.
    // Dps310PressureSensor.begin(Wire, 0x76);
    // Use the commented line below instead of the one above to use the default I2C address.
    // if you are using the Pressure 3 click Board, you need 0x76
    Dps310PressureSensor.begin(Wire, 0x77);
    Serial.println("Init complete!");
    test_status = DSP310_TEST;
    break;
  }
  case DSP310_TEST:
  {
    dsp310_fetch();
    break;
  }
  case DEEPSLEEP_BY_GPIO_TEST:
  {
    delay(1000);
#define DEEPSLEEP_BY_GPIO_TEST_PIN 4
#define SX1262_RST 5
    pinMode(DEEPSLEEP_BY_GPIO_TEST_PIN, INPUT);
    pinMode(SX1262_RST, OUTPUT);
    digitalWrite(SX1262_RST, LOW);
    esp_deep_sleep_enable_gpio_wakeup(1 << DEEPSLEEP_BY_GPIO_TEST_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
    break;
  }
  case DEEPSLEEP_BY_TIMER_TEST:
  {
    uint64_t time_to_sleep_us = 10 * 1000000;
    esp_sleep_enable_timer_wakeup(time_to_sleep_us);
    esp_deep_sleep_start();
    break;
  }
  case LED_INIT:
  {
    led_init();
    test_status = LED_TEST;
    break;
  }
  case LED_TEST:
  {
    digitalWrite(pVext, HIGH);
    delay(1000);
    led_test();
    digitalWrite(pVext, LOW);
    delay(1000);
    break;
  }
  case RS485_INIT:
  {
    rs485_init();
    test_status = RS485_TEST;
    break;
  }
  case RS485_TEST:
  {
    rs485_status_handle();
    break;
  }
  case BAT_TEST:
  {
    float batteryVoltage = getBatteryVoltage();
    String battery_power = "BAT: " + (String)batteryVoltage;
    getBatteryLevel();
    break;
  }
  default:
    break;
  }
}
