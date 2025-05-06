#include <Arduino.h>
#include <SPI.h>
#include "constants.hpp"
#include "shared.hpp"
#include "amt22.hpp"
#include "bno085.hpp"

static SPISettings motorSettings(10000000, MSBFIRST, SPI_MODE0);
extern SharedState SHARED;

static AMT22 encA(CS_ENC_A);
static AMT22 encB(CS_ENC_B);
static ImuBNO085 imu;

static void motor_write_vel(float v0, float v1) {
  // TODO: encode velocities into driver frame and shift out over SPI with CS_MOTOR
}

void io_task(void *param) {
  const TickType_t ts = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  imu.begin(CS_IMU);

  while (true) {
    // --- E‑stop + watchdog ---
    if (millis() - SHARED.last_cmd_ms > WDOG_MS) SHARED.estop = true;

    encA.poll();
    encB.poll();

    SHARED.enc_pos[0] = encA.angleRad();
    SHARED.enc_vel[0] = encA.velocityRadS();
    SHARED.enc_pos[1] = encB.angleRad();
    SHARED.enc_vel[1] = encB.velocityRadS();

    imu.pollInto(SHARED);      // <-- SINGLE call, no manual copies

    // ---------- actuators ----------------
    if (SHARED.estop)  motor_write_vel(0,0);
    else               motor_write_vel(SHARED.cmd_vel[0], SHARED.cmd_vel[1]);

    vTaskDelay(ts);
  }
}