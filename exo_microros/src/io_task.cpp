#include <Arduino.h>
#include <SPI.h>
#include "constants.hpp"
#include "shared.hpp"
#include "amt22.hpp"
#include "bno085.hpp"
#include "escon_pwm.hpp"

static SPISettings motorSettings(10000000, MSBFIRST, SPI_MODE0);
extern SharedState SHARED;

static AMT22 encHip(CS_ENC_A);
static AMT22 encKnee(CS_ENC_B);
static ImuBNO085 imu;
static EsconPwmMotor motorHip (PWM_HIP,  DIR_HIP,  EN_HIP);
static EsconPwmMotor motorKnee(PWM_KNEE, DIR_KNEE, EN_KNEE);

void io_task(void *param) {
  SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  imu.begin(CS_IMU);
  encHip.begin();
  encKnee.begin();
  motorHip .begin(0);
  motorKnee.begin(1);

  const TickType_t ts = pdMS_TO_TICKS(1000 / CONTROL_HZ);

  while (true) {
    // sensors
    encHip.poll();
    encKnee.poll();
    imu.pollInto(SHARED);

    SHARED.enc_pos[0] = encHip.angleRad();
    SHARED.enc_vel[0] = encHip.velocityRadS();
    SHARED.enc_pos[1] = encKnee.angleRad();
    SHARED.enc_vel[1] = encKnee.velocityRadS();

    float hip_cmd, knee_cmd;
    uint32_t t_cmd;
    bool estop, enable;
    portENTER_CRITICAL(&g_shared_mux);
      hip_cmd  = SHARED.cmd_vel[0];
      knee_cmd = SHARED.cmd_vel[1];
      t_cmd    = SHARED.last_cmd_ms;
      estop    = SHARED.estop;
      enable   = SHARED.enable;
      if (millis() - t_cmd > WDOG_MS) SHARED.estop = true;
    portEXIT_CRITICAL(&g_shared_mux);

    // actuators 
    if (estop || !enable)
    {
      motorHip.coast();
      motorKnee.coast();
    }
    else
    {
      motorHip.enable(true);
      motorKnee.enable(true);
      motorHip .setCmd(hip_cmd  / MAX_RPM_MOTOR);
      motorKnee.setCmd(knee_cmd / MAX_RPM_MOTOR);
    }

    vTaskDelay(ts);
  }
}