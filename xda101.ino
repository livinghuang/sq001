#include "global.h"

void setup()
{
  Serial.begin(115200);
  // generate_lorawan_parameters_by_chip_id();
  esp_sleep_wakeup_cause_t wakeup_reason = print_wakeup_reason();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    get_chip_id();
    // ble_init();
    // run_with_time_escape(30000, ble_process, ble_stop);
  }
  lorawan_init();
}

void loop()
{
  lorawan_process();
}
