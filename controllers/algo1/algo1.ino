#ifndef ARDUINO
#include "Arduino.hpp"
#include "Mona_ESP_lib.hpp"
#else
#include "Mona_ESP_lib.h"
#include <Wire.h>
#endif

void setup()
{
  Mona_ESP_init();
  Serial.begin(115200);
  Motors_forward(100);
}

void loop()
{
}
