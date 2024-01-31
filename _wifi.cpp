#include "global.h"
uint16_t wifi_connect_try_num = 15;
void wifi_connect_init(void)
{
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin("xxxxxx", "xxxxxx"); // fill in "Your WiFi SSID","Your Password"
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
