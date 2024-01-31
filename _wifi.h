#ifndef WIFI_H
#define WIFI_H
#include "global.h"

extern uint16_t wifi_connect_try_num;

void wifi_connect_init(void);
bool wifi_connect_try(uint8_t try_num);
void wifi_scan(unsigned int value);
#endif