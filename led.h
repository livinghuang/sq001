#define _LED_H
#ifdef _LED_H
#include "bsp.h"
#include "esp32-hal-rgb-led.h"
// Define the GPIO pin where the WS2812 LED data input is connected
#define LED_PIN 4 // Replace with your GPIO pin number

enum LedColor
{
  RED,
  GREEN,
  BLUE,
  WHITE,
  OFF
};
void led(enum LedColor color);
void led_init(void);
void led_test(void);
#endif