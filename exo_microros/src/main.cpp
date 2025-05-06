#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "constants.hpp"
#include "shared.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This is for serial only currently
#endif

SharedState SHARED;

static rcl_publisher_t  pub_joint, pub_imu, pub_estop, pub_calib;
static rcl_subscription_t sub_cmd, sub_joy;
static rclc_executor_t   executor;

static sensor_msgs__msg__JointState js_msg;
static sensor_msgs__msg__Imu       imu_msg;
static std_msgs__msg__Bool         estop_msg;
static std_msgs__msg__UInt8        calib_msg;
static sensor_msgs__msg__JointState * cmd_in_msg = nullptr;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
[[noreturn]] void error_loop() {
  while(1) {
    delay(100);
  }
}

void cmd_cb(const void * msg_in) {
  const auto * msg = static_cast<const sensor_msgs__msg__JointState*>(msg_in);
  if (msg->velocity.size >= 2) {
    SHARED.cmd_vel[0] = msg->velocity.data[0];
    SHARED.cmd_vel[1] = msg->velocity.data[1];
    SHARED.last_cmd_ms = millis();
  }
}

// forward declare for freeRTOS task
void io_task(void *);

void setup() {
  // Configure serial transport
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000);

  Serial.println("setup");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // publishers
  RCCHECK(rclc_publisher_init_default(
    &pub_joint, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));
  RCCHECK(rclc_publisher_init_default(
    &pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));
  RCCHECK(rclc_publisher_init_default(
    &pub_estop, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/estop"));
  RCCHECK(rclc_publisher_init_default(
    &pub_calib, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/imu/calib"));

  // subscribers
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_velocity_cmd"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  cmd_in_msg = sensor_msgs__msg__JointState__create();   // heap once
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_in_msg, &cmd_cb, ON_NEW_DATA));
  sensor_msgs__msg__JointState__init(&js_msg);
  rosidl_runtime_c__double__Sequence__init(&js_msg.position, 2);
  rosidl_runtime_c__double__Sequence__init(&js_msg.velocity, 2);
  sensor_msgs__msg__Imu__init(&imu_msg);   // all fields zeroed

  // create task
  xTaskCreatePinnedToCore(io_task, "io_task", 4096, NULL, 2, NULL, 1);
  Serial.println("setup done");
}

void loop() {
  Serial.println("loop");

  js_msg.header.stamp.sec  = millis() / 1000;
  js_msg.header.stamp.nanosec = (millis() % 1000) * 1000000u;

  js_msg.position.data[0]  = SHARED.enc_pos[0];
  js_msg.position.data[1]  = SHARED.enc_pos[1];
  js_msg.velocity.data[0]  = SHARED.enc_vel[0];
  js_msg.velocity.data[1]  = SHARED.enc_vel[1];
  RCSOFTCHECK(rcl_publish(&pub_joint, &js_msg, nullptr));

  imu_msg.header = js_msg.header;          // reuse timestamp

  imu_msg.orientation.w = SHARED.imu.quat[0];
  imu_msg.orientation.x = SHARED.imu.quat[1];
  imu_msg.orientation.y = SHARED.imu.quat[2];
  imu_msg.orientation.z = SHARED.imu.quat[3];

  imu_msg.angular_velocity.x = SHARED.imu.gyro[0];
  imu_msg.angular_velocity.y = SHARED.imu.gyro[1];
  imu_msg.angular_velocity.z = SHARED.imu.gyro[2];

  imu_msg.linear_acceleration.x = SHARED.imu.accel[0];
  imu_msg.linear_acceleration.y = SHARED.imu.accel[1];
  imu_msg.linear_acceleration.z = SHARED.imu.accel[2];

  RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, nullptr));

  estop_msg.data = SHARED.estop;
  calib_msg.data = SHARED.imu.calib;
  RCSOFTCHECK(rcl_publish(&pub_estop,  &estop_msg,  nullptr));
  RCSOFTCHECK(rcl_publish(&pub_calib,  &calib_msg,  nullptr));

  rclc_executor_spin_some(&executor, /*timeout_ns*/ 0);
}