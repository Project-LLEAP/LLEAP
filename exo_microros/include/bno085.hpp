#pragma once
#include <Arduino.h>

#include <Adafruit_BNO08x.h>
#include "shared.hpp"

class ImuBNO085 {
 public:
  bool begin(uint8_t cs, uint8_t int_pin = 255);
  bool poll();
  const ImuFrame& data() const { return frame_; }

  bool pollInto(SharedState& dst);

 private:
  Adafruit_BNO08x bno_;
  volatile ImuFrame frame_{};
  static portMUX_TYPE mux_;
};