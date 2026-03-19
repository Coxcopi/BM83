#include <Arduino.h>
#include "BM83.h"
#include "driver/gpio.h"

BM83 bm83 = BM83();

void setup() {
  delay(500);
  uint8_t params[2] = {0x00, 0x51};
  uint8_t params2[2] = {0x00, 0x52};
  bm83.begin(GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_NC);
  delay(1000);
  bm83.send(0x02, params, sizeof(params));
  bm83.send(0x02, params2, sizeof(params2));
}

void loop() {
}

