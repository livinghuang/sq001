#define pVext 1
#define pADC_BAT 2
#define pSDA 18
#define pSCL 19
#define pLORA_RST 5

// RS485
#define RO485_RO 19 // Receiver Output (RO) connected to pin 19
#define RO485_DI 18 // Driver Input (DI) connected to pin 18
#define RO485_DE 4  // Data Enable (DE) connected to pin 4

// 24 POWER
#define pSDN 3

void power_On_Sensor_Bus();
void power_Off_Sensor_Bus();
void enter_deepsleep(void);