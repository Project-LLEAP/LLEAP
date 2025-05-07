#pragma once
#include <unity.h>
#include "test_fakes.hpp"
#include "shared.hpp"

extern SharedState SHARED;

inline void enable_reset()
{
  SHARED.enable = false;
  fake_ms       = 0;
  SHARED.last_cmd_ms = 123;
}

inline void test_enable_goes_true()
{
  enable_reset();
  std_msgs__msg__Bool m; m.data = true;
  extern void enable_cb(const void*);
  enable_cb(&m);
  TEST_ASSERT_TRUE(SHARED.enable.load());
}

inline void test_disable_clears_watchdog()
{
  enable_reset();
  std_msgs__msg__Bool m; m.data = false;
  extern void enable_cb(const void*);
  enable_cb(&m);
  TEST_ASSERT_FALSE(SHARED.enable.load());
  TEST_ASSERT_EQUAL_UINT32(fake_ms, SHARED.last_cmd_ms);
}
