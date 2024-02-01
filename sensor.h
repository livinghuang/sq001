#define _SENSOR_H
#ifdef _SENSOR_H
#include "global.h"

extern HDC1080 hdc1080;

bool hdc1080_fetch(void);
void fetchSensorData(void);
#endif