#include <unity.h>

#include "test_fakes.hpp"
#include "test_estop.hpp"
#include "test_enable.hpp"

// stub implementations
void estop_cb(const void * msg_in)
{
  auto * b = static_cast<const std_msgs__msg__Bool*>(msg_in);
  SHARED.estop.store(b->data, std::memory_order_relaxed);
  if (!b->data) SHARED.last_cmd_ms = millis();   // refresh watchdog
}

void enable_cb(const void * msg_in)
{
  auto * b = static_cast<const std_msgs__msg__Bool*>(msg_in);
  if (!b->data) { 
    SHARED.last_cmd_ms = millis();
    SHARED.enable.store(b->data, std::memory_order_relaxed);
  }
}

void setUp(void)   {} 
void tearDown(void) {}

int main()
{
  UNITY_BEGIN();

  // estop suite
  RUN_TEST(test_estop_cannot_clear);
  RUN_TEST(test_estop_asserts);

  // enable suite
  RUN_TEST(test_enable_goes_true);
  RUN_TEST(test_disable_clears_watchdog);

  return UNITY_END();
}
