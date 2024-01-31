#define _SENSOR_H
#ifdef _SENSOR_H
#include "global.h"

extern BMP280 bmp;
extern HDC1080 hdc1080;
extern Dps310 Dps310PressureSensor;

bool hdc1080_fetch(void);
bool bmp280_fetch(void);
bool dsp310_fetch(void);
void fetchSensorData(void);
#endif