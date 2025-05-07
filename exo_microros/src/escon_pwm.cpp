#include "escon_pwm.hpp"

EsconPwmMotor::EsconPwmMotor(int pwm_pin, int dir_pin, int en_pin)
: pwm_(pwm_pin), dir_(dir_pin), en_(en_pin), ledc_chan_(255) {}

void EsconPwmMotor::begin(uint8_t ledc_chan)
{
    ledc_chan_ = ledc_chan;

    pinMode(dir_, OUTPUT);
    if (en_ >= 0)  pinMode(en_, OUTPUT);

        
    ledcSetup(ledc_chan_, 20000, 13);       // 13-bit PWM
    ledcAttachPin(pwm_, ledc_chan_);
    setDuty(0);
    enable(false);
}

void EsconPwmMotor::enable(bool on)
{
    if (en_ >= 0) digitalWrite(en_, on ? HIGH : LOW);
}

void EsconPwmMotor::coast()
{
    setDuty(0);
    enable(false);
}

void EsconPwmMotor::setCmd(float cmd)
{
    cmd = constrain(cmd, -1.0f, 1.0f);
    digitalWrite(dir_, cmd >= 0.0f ? HIGH : LOW);
    setDuty(fabsf(cmd));
}

void EsconPwmMotor::setDuty(float frac)
{
    uint32_t duty = uint32_t(frac * ((1 << 13) - 1));   // 0-8191
    ledcWrite(ledc_chan_, duty);
}
