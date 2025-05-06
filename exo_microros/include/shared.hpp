#pragma once
#include <stdint.h>

struct ImuFrame {
  float quat[4];
  float gyro[3];
  float accel[3];
  float mag[3];
  uint8_t calib;
};

struct SharedState {
  float cmd_vel[2]{};
  float enc_pos[2]{};
  float enc_vel[2]{};
  float joy_vel[2]{};
  ImuFrame imu{};
  volatile uint32_t last_cmd_ms{0};
  uint32_t last_joy_ms{0};
  volatile bool estop{false};
};