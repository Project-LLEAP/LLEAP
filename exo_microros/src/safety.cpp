#include <Arduino.h>
#include "constants.hpp"
#include "shared.hpp"

extern SharedState SHARED;

static void IRAM_ATTR estop_isr() {
  SHARED.estop = (digitalRead(PIN_ESTOP) == LOW);
}

void safety_init() {
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estop_isr, CHANGE);
}