#pragma once
#include <Arduino.h>

// VSPI default pins are fixed (CLK 18, MISO 19, MOSI 23)
constexpr int PIN_SPI_CLK = 18;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_MOSI = 23;
constexpr int CS_MOTOR  = 5;
constexpr int CS_ENC_A  = 17;
constexpr int CS_ENC_B  = 16;
constexpr int CS_IMU    = 4;
constexpr int PIN_ESTOP = 15;

constexpr float PI_2 = PI * 2.0f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float RAD2DEG = 180.0f / PI;

constexpr float ENCODER_RES = 16384.0f;

constexpr int CONTROL_HZ = 200;
constexpr int WDOG_MS = 1000 / CONTROL_HZ;

constexpr int AXIS_FWD  = 1;              // left stick vertical
constexpr int AXIS_TURN = 0;              // left stick horizontal
constexpr int BTN_ENABLE = 4;             // LB on XBox pad
constexpr int BTN_ESTOP  = 5;             // RB

constexpr float MAX_VEL_RAD_S = 4.0f;     // scale |axis|≤1 → ±4 rad/s per joint

void error_loop();