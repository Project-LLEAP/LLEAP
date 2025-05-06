#include "bno085.hpp"

portMUX_TYPE ImuBNO085::mux_ = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------------------------
bool ImuBNO085::begin(uint8_t cs, uint8_t int_pin)
{
  if (!bno_.begin_SPI(cs, int_pin)) return false;

  // 200 Hz  (interval = µs)
  bno_.enableReport(SH2_ROTATION_VECTOR,        5000);
  bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
  bno_.enableReport(SH2_ACCELEROMETER,          5000);
  bno_.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED,         5000);
  return true;
}

// ------------------------------------------------------------------
bool ImuBNO085::poll()
{
  sh2_SensorValue_t v;
  if (!bno_.getSensorEvent(&v)) return false;

  portENTER_CRITICAL(&mux_);
  switch (v.sensorId) {
    case SH2_ROTATION_VECTOR:
      frame_.quat[0] = v.un.rotationVector.real;
      frame_.quat[1] = v.un.rotationVector.i;
      frame_.quat[2] = v.un.rotationVector.j;
      frame_.quat[3] = v.un.rotationVector.k;
      frame_.calib   = v.status & 0x03;
      break;

    case SH2_GYROSCOPE_UNCALIBRATED:
      frame_.gyro[0] = v.un.gyroscope.x;
      frame_.gyro[1] = v.un.gyroscope.y;
      frame_.gyro[2] = v.un.gyroscope.z;
      break;

    case SH2_ACCELEROMETER:
      frame_.accel[0] = v.un.accelerometer.x;
      frame_.accel[1] = v.un.accelerometer.y;
      frame_.accel[2] = v.un.accelerometer.z;
      break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
      frame_.mag[0] = v.un.magneticField.x;
      frame_.mag[1] = v.un.magneticField.y;
      frame_.mag[2] = v.un.magneticField.z;
      break;
  }
  portEXIT_CRITICAL(&mux_);
  return true;
}

bool ImuBNO085::pollInto(SharedState& dst)
{
  if (!poll()) return false;
  portENTER_CRITICAL(&mux_);
  dst.imu = frame_;          // single 64-byte struct copy
  portEXIT_CRITICAL(&mux_);
  return true;
}