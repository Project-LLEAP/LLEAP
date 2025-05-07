#pragma once
#include <cstdint>
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* ---------- Fake ROS message types ---------- */
struct std_msgs__msg__Bool {
  bool data;
};

/* ---------- Fake millis/micros ---------- */
extern uint32_t fake_ms;
inline uint32_t millis() { return fake_ms; }

/* ---------- Fake critical section macros ---------- */
#define portENTER_CRITICAL(x)   (void)x
#define portEXIT_CRITICAL(x)    (void)x
#define portENTER_CRITICAL_ISR(x) (void)x
#define portEXIT_CRITICAL_ISR(x)  (void)x
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0

// Global shared mutex for critical sections
extern portMUX_TYPE g_shared_mux;
