#pragma once
#include <Arduino.h>

class EsconPwmMotor {
public:
  /**
   * @param pwm_pin  LEDC-capable GPIO
   * @param dir_pin  Direction GPIO
   * @param en_pin   Enable GPIO (-1 if permanently tied high)
   */
  EsconPwmMotor(int pwm_pin, int dir_pin, int en_pin = -1);

  void begin(uint8_t ledc_chan = 0);

  /// TRUE → enable (output stage active), FALSE → disable / coast
  void enable(bool on);

  /// Immediately disable and drive PWM = 0  (free-wheel / no resist)
  void coast();

  /**
   * @param cmd  −1 … +1    (full reverse … full forward)
   * Internally maps sign to DIR and magnitude to 13-bit PWM duty.
   */
  void setCmd(float cmd);

private:
  void setDuty(float frac);          // 0…1 → 0…8191

  int pwm_, dir_, en_;
  uint8_t ledc_chan_;
};