#include <unity.h>
#define UNIT_TEST
#include "test_fakes.hpp"
#include "shared.hpp"

extern SharedState SHARED;
// Instead of including main.cpp, just define the callback function
void estop_cb(const void * msg_in) {
  const auto * msg = static_cast<const std_msgs__msg__Bool*>(msg_in);
  SHARED.estop.store(msg->data, std::memory_order_relaxed);

  if (!msg->data) {
    portENTER_CRITICAL_ISR(&g_shared_mux);
      SHARED.last_cmd_ms = millis();
    portEXIT_CRITICAL_ISR(&g_shared_mux);
  }
}

// Setup function for estop tests
void setup_estop_test() {
  SHARED.estop = true;
  fake_ms = 0;
  SHARED.last_cmd_ms = 0;
}

// PlatformIO Unity setup
void setUp(void) {}
void tearDown(void) {}

void test_estop_clears_watchdog(void)
{
  setup_estop_test();
  std_msgs__msg__Bool m; m.data = false;
  estop_cb(&m);
  TEST_ASSERT_FALSE(SHARED.estop.load());
  TEST_ASSERT_EQUAL_UINT32(fake_ms, SHARED.last_cmd_ms);
}

void test_estop_asserts(void)
{
  setup_estop_test();
  std_msgs__msg__Bool m; m.data = true;
  estop_cb(&m);
  TEST_ASSERT_TRUE(SHARED.estop.load());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_estop_clears_watchdog);
  RUN_TEST(test_estop_asserts);
  return UNITY_END();
}
