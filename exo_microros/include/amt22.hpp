#pragma once
#include <Arduino.h>
#include <SPI.h>

class AMT22 {
public:
  explicit AMT22(uint8_t cs_pin);

  void begin();
  
  bool poll();

  float angleRad() const;
  float velocityRadS() const;
  
private:
  static constexpr float    INV_LSB   = 2.0f * PI / 16384.0f; // 2π/2¹⁴

  bool readRaw(uint16_t &raw);
  
  uint8_t cs_pin_;
  uint16_t last_raw_;
  uint32_t last_us_;
  float angle_;
  float vel_;
  static portMUX_TYPE mux_;
};
