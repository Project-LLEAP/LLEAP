#pragma once
#include <unity.h>
#include "test_fakes.hpp"
#include "shared.hpp"

extern SharedState SHARED;

inline void estop_reset()
{
  SHARED.estop = true;
  fake_ms      = 0;
  SHARED.last_cmd_ms = 0;
}

inline void test_estop_cannot_clear()
{
  estop_reset();
  std_msgs__msg__Bool m; m.data = true;
  extern void estop_cb(const void*);   // from main firmware
  estop_cb(&m);
  m.data = false;
  estop_cb(&m); // attempt clear
  TEST_ASSERT_TRUE(SHARED.estop.load());
}

inline void test_estop_asserts()
{
  estop_reset();
  std_msgs__msg__Bool m; m.data = true;
  extern void estop_cb(const void*);
  estop_cb(&m);
  TEST_ASSERT_TRUE(SHARED.estop.load());
}
