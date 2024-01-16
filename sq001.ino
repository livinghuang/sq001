#include "bsp.h"
#include "WiFi.h"
#include "Wire.h"
#include "BMP280.h"
#include "HDC1080.h"
#include "esp_system.h"
#include "LoRaWan_APP.h"
#include "led.h"
#include "esp_sleep.h"

bool resendflag = false;
bool deepsleepflag = false;
bool interrupt_flag = false;

RTC_DATA_ATTR int bootCount = 0;

uint64_t chipid;

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
  DEEPSLEEP_BY_TIMER_TEST,
  DEEPSLEEP_BY_GPIO_TEST,
  LED_INIT,
  LED_TEST,
  BAT_TEST,
  _TEST_INIT,
  _TEST,
} test_status_t;

test_status_t test_status;

uint16_t wifi_connect_try_num = 15;

BMP280 bmp;
HDC1080 hdc1080;
float getBatVolt();
uint8_t GetBatteryLevel(void);
struct hdc1080_data
{
  float temperature;
  float humidity;
} hdc1080_result;

struct bmp280_data
{
  float bmp280_internal_temperature;
  float pressure;
} bmp280_result;

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

bool bmp280_fetch(void)
{
  if (!bmp.begin())
  {
    return 0;
  }
  bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                  BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  BMP280::FILTER_X16,      /* Filtering. */
                  BMP280::STANDBY_MS_500); /* Standby time. */
  delay(20);
  float temp = bmp.readTemperature();
  float Pressure = (float)bmp.readPressure() / 100.0;
  int c = 0;
  while ((temp < -50) || (Pressure > 1100) || (Pressure < 500))
  {
    bmp.putBMP280ToSleep();
    delay(10);
    bmp.end();
    Serial.println("BMP ERROR");
    Serial.flush();
    bmp.begin();
    delay(10);
    bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                    BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    BMP280::FILTER_X16,      /* Filtering. */
                    BMP280::STANDBY_MS_500); /* Standby time. */
    temp = bmp.readTemperature();
    Pressure = (float)bmp.readPressure() / 100.0;
    delay(10);
    c++;
    if (c > 3)
    {
      return false;
    }
  }
  bmp.putBMP280ToSleep();
  delay(10);
  bmp.end();
  Serial.printf("T=%.2f degC, Pressure=%.2f hPa\n", temp, Pressure);
  bmp280_result.bmp280_internal_temperature = temp;
  bmp280_result.pressure = Pressure;
  return true;
}

/********************************* lorawan  *********************************************/
/* OTAA para*/
uint8_t devEui[] = {0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};

/* ABP para*/
uint8_t nwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t appSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;

static void prepareTxFrame(uint8_t port)
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
   *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
   *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
   *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
   *for example, if use REGION_CN470,
   *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
   */
  appDataSize = 4;
  appData[0] = 0x00;
  appData[1] = 0x01;
  appData[2] = 0x02;
  appData[3] = 0x03;
}

void lorawan_init(void)
{
  // fetchSensorData();
  deviceState = DEVICE_STATE_INIT;
  Serial.flush();
}

void lorawan_status_handle()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
    LoRaWAN.init(loraWanClass, loraWanRegion);
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    prepareTxFrame(appPort);
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    // Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    LoRaWAN.sleep(loraWanClass);
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}
/********************************* lorawan  *********************************************/

/********************************* lora  *********************************************/
#define RF_FREQUENCY 868000000 // Hz

#define TX_OUTPUT_POWER 10 // dBm

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum
{
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

int16_t txNumber = 0;
int16_t rxNumber = 0;
States_t state;
bool sleepMode = false;
int16_t Rssi, rxSize;

String rssi = "RSSI --";
String packet;
String send_num;

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;    // last send time
int interval = 1000;      // interval between sends
int16_t RssiDetection = 0;

void OnTxDone(void)
{
  Serial.print("TX done......");
  state = STATE_RX;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.print("TX Timeout......");
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rxNumber++;
  Rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, Rssi, rxSize);
  Serial.println("wait to send next packet");
  receiveflag = true;
  state = STATE_TX;
}
void lora_init(void)
{
  txNumber = 0;
  Rssi = 0;
  rxNumber = 0;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  state = STATE_TX;
  Serial.println("waiting lora data!");
}
void lora_status_handle(void)
{
  if (resendflag)
  {
    state = STATE_TX;
    resendflag = false;
  }

  if (receiveflag && (state == LOWPOWER))
  {
    receiveflag = false;
    packet = "Rdata:";
    int i = 0;
    while (i < rxSize)
    {
      packet += rxpacket[i];
      i++;
    }
    String packSize = "R_Size:";
    packSize += String(rxSize, DEC);
    packSize += " R_rssi:";
    packSize += String(Rssi, DEC);
    send_num = "send num:";
    send_num += String(txNumber, DEC);
    delay(100);
    Serial.println(packet);
    Serial.println(packSize);
    Serial.println(send_num);
    if ((rxNumber % 2) == 0)
    {
      // digitalWrite(LED, HIGH);
    }
  }
  switch (state)
  {
  case STATE_TX:
    delay(1000);
    txNumber++;
    sprintf(txpacket, "hello %d,Rssi:%d", txNumber, Rssi);
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    state = LOWPOWER;
    break;
  case STATE_RX:
    Serial.println("into RX mode");
    Radio.Rx(0);
    state = LOWPOWER;
    break;
  case LOWPOWER:
    Radio.IrqProcess();
    break;
  default:
    break;
  }
}
/********************************* lora  *********************************************/

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

void fetchSensorData()
{
  power_On_Sensor_Bus();
  delay(5);
  while (1)
  {
    bmp280_fetch();
    hdc1080_fetch();
    delay(1000);
  }
  power_Off_Sensor_Bus();
}

/********************************* WIFI  *********************************************/

void wifi_connect_init(void)
{
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin("Living_Deco", "asdfghjklzzz"); // fill in "Your WiFi SSID","Your Password"
  Serial.println("WIFI Setup done");
}

bool wifi_connect_try(uint8_t try_num)
{
  uint8_t count;
  while (WiFi.status() != WL_CONNECTED && count < try_num)
  {
    count++;
    Serial.println("wifi connecting...");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("wifi connect OK");
    delay(2500);
    return true;
  }
  else
  {
    Serial.println("wifi connect failed");
    delay(1000);
    return false;
  }
}

void wifi_scan(unsigned int value)
{
  unsigned int i;
  WiFi.disconnect(); //
  WiFi.mode(WIFI_STA);
  for (i = 0; i < value; i++)
  {
    Serial.println("Scan start...");
    int n = WiFi.scanNetworks();
    Serial.println("Scan done");
    Serial.flush();
    if (n == 0)
    {
      Serial.println("no network found");
      delay(2000);
    }
    else
    {
      Serial.printf("Found %d networks:", n);
      Serial.flush();
      for (int i = 0; (i < n); i++)
      {
        Serial.println((String)(WiFi.SSID(i)) + " (" + (String)(WiFi.RSSI(i)) + ")");
        Serial.flush();
      }
      delay(100);
    }
  }
}

/********************************* WIFI  *********************************************/

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

void interrupt_GPIO9(void)
{
  interrupt_flag = true;
}

void interrupt_handle(void)
{
  if (interrupt_flag)
  {
    interrupt_flag = false;
    Serial.println("interrupt handle");
    Serial.flush();
    delay(1000);
  }
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}
void setup()
{
  Serial.begin(115200);
  Mcu.begin();

  print_wakeup_reason();
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  Serial.flush();
  delay(1000);

  resendflag = false;
  deepsleepflag = false;
  interrupt_flag = false;

  chipid = ESP.getEfuseMac();                                  // The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32ChipID=%04X", (uint16_t)(chipid >> 32)); // print High 2 bytes
  Serial.printf("%08X\n", (uint32_t)chipid);                   // print Low 4bytes.
  Serial.flush();
  delay(1000);
  test_status = LORA_TEST_INIT;
}

void loop()
{
  interrupt_handle();
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
    break;
  }
  case LORAWAN_COMMUNICATION_TEST:
  {
    lorawan_status_handle();
    break;
  }
  case LORA_TEST_INIT:
  {
    Serial.println("LORA_TEST_INIT");
    Serial.flush();
    delay(1000);
    lora_init();
    test_status = LORAWAN_COMMUNICATION_TEST;
    break;
  }
  case LORA_COMMUNICATION_TEST:
  {
    Serial.println("LORA_COMMUNICATION_TEST");
    Serial.flush();
    delay(1000);
    lora_status_handle();
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
  case BAT_TEST:
  {
    float batteryVoltage = getBatVolt();
    String battery_power = "BAT: " + (String)batteryVoltage;
    GetBatteryLevel();
    break;
  }
  default:
    break;
  }
}

float getBatVolt()
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

uint8_t GetBatteryLevel(void)
{
  const float maxBattery = 4.2;
  const float minBattery = 3.0;
  const float batVolt = getBatVolt();
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
