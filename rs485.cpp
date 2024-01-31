#include "global.h"
void rs485_init(void)
{
  pinMode(pVext, OUTPUT);
  pinMode(pSDN, OUTPUT);
  digitalWrite(pSDN, LOW);

  pinMode(RO485_RO, INPUT);  // 将RO引脚配置为输入
  pinMode(RO485_DI, OUTPUT); // 将DI引脚配置为输出
  pinMode(RO485_DE, OUTPUT);
  digitalWrite(RO485_DE, LOW); // Disable transmission initially
  Serial1.begin(9600, SERIAL_8N1, RO485_RO, RO485_DI);
  bool extern_24v = true;
  if (extern_24v == true)
  {
    digitalWrite(pVext, HIGH);
    digitalWrite(pSDN, HIGH);
  }
}

void rs485_status_handle(void)
{
#define device_address 0x01
  byte modbusQuery[] = {device_address, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C}; // Example Modbus query
  Serial.print("Sent:");
  printHex(modbusQuery, sizeof(modbusQuery));
  digitalWrite(RO485_DE, HIGH); // Enable transmission
  Serial1.write(modbusQuery, sizeof(modbusQuery));
  Serial1.flush();
  digitalWrite(RO485_DE, LOW); // Disable transmission
  while (1)
  {
    byte responseBuffer[256]; // Adjust the buffer size as needed
    int bytesRead = Serial1.readBytes(responseBuffer, sizeof(responseBuffer));

    if (bytesRead > 0)
    {
      // Process the received bytes here
      Serial.print("Received:");
      if (responseBuffer[0] == device_address)
      {
        printHex(responseBuffer, bytesRead);
      }
      else
      {
        Serial.println("No response received.");
      }
      break;
    }
    delay(100);
  }
  Serial.flush();
  delay(1000);
}