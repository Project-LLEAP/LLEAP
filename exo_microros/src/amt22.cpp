#include "amt22.hpp"

SPISettings encSettings(10000000, MSBFIRST, SPI_MODE1);
portMUX_TYPE AMT22::mux_ = portMUX_INITIALIZER_UNLOCKED;

AMT22::AMT22(uint8_t cs) : cs_pin_(cs), last_raw_(0) {}

void AMT22::begin()
{
  last_us_ = micros();
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);
}

bool AMT22::readRaw(uint16_t &raw)
{
  SPI.beginTransaction(encSettings);
  digitalWrite(cs_pin_, LOW);
  delayMicroseconds(3);
  SPI.transfer16(0x0000);          // NOP
  digitalWrite(cs_pin_, HIGH);
  delayMicroseconds(20);
  digitalWrite(cs_pin_, LOW);
  raw = SPI.transfer16(0x0000);
  digitalWrite(cs_pin_, HIGH);
  SPI.endTransaction();

  // parity (even) = XOR of 14 data bits
  uint16_t data = raw >> 2;
  if (__builtin_parity(data) != ((raw >> 1) & 1)) return false;
  raw &= 0x3FFF;
  return true;
}

bool AMT22::poll()
{
  uint16_t raw;
  if (!readRaw(raw)) return false;

  uint32_t now = micros();
  float dt = (now - last_us_) * 1e-6f;
  if (dt < 1e-6f) dt = 1e-6f;              // avoid div-0 on first sample

  // unwrap (shortest path) across 0-16383 boundary
  int16_t diff = static_cast<int16_t>(raw - last_raw_);
  if (diff >  8192) diff -= 16384;
  if (diff < -8192) diff += 16384;

  portENTER_CRITICAL(&mux_);
  angle_ = raw * INV_LSB;
  vel_   = diff * INV_LSB / dt;
  portEXIT_CRITICAL(&mux_);

  last_raw_ = raw;
  last_us_  = now;
  return true;
}

float AMT22::angleRad() const
{
  portENTER_CRITICAL(&mux_);
  float a = angle_;
  portEXIT_CRITICAL(&mux_);
  return a;
}

float AMT22::velocityRadS() const
{
  portENTER_CRITICAL(&mux_);
  float v = vel_;
  portEXIT_CRITICAL(&mux_);
  return v;
}