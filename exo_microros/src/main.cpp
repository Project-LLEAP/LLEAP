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
static rcl_subscription_t sub_cmd, sub_estop;
static rclc_executor_t   executor;

static sensor_msgs__msg__JointState js_msg;
static sensor_msgs__msg__Imu       imu_msg;
static std_msgs__msg__Bool         estop_msg;
static std_msgs__msg__UInt8        calib_msg;
static sensor_msgs__msg__JointState * cmd_in_msg = nullptr;
static std_msgs__msg__Bool         estop_in_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
[[noreturn]] void error_loop() {
  while(1) {
    delay(100);
  }
}

void estop_cb(const void * msg_in) {
  const auto * msg = static_cast<const std_msgs__msg__Bool*>(msg_in);
  SHARED.estop.store(msg->data, std::memory_order_relaxed);

  if (!msg->data) {
    portENTER_CRITICAL_ISR(&g_shared_mux);
      SHARED.last_cmd_ms = millis();
    portEXIT_CRITICAL_ISR(&g_shared_mux);
  }
}

void cmd_cb(const void * msg_in) {
  const auto * msg = static_cast<const sensor_msgs__msg__JointState*>(msg_in);
  if (msg->velocity.size >= 2) {
    portENTER_CRITICAL_ISR(&g_shared_mux);
    SHARED.cmd_vel[0] = msg->velocity.data[0];
    SHARED.cmd_vel[1] = msg->velocity.data[1];
    SHARED.last_cmd_ms = millis();
    portEXIT_CRITICAL_ISR(&g_shared_mux);
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
  RCCHECK(rclc_subscription_init_default(
    &sub_estop, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/estop"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  cmd_in_msg = sensor_msgs__msg__JointState__create();   // heap once
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, cmd_in_msg, &cmd_cb, ON_NEW_DATA));
  std_msgs__msg__Bool__init(&estop_in_msg);
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_estop, &estop_in_msg, &estop_cb, ON_NEW_DATA));
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

  // read shared state
  float  pos[2], vel[2];
  ImuFrame imu_local;
  bool   estop_local;
  uint8_t calib_local;

  portENTER_CRITICAL(&g_shared_mux);
    pos[0]   = SHARED.enc_pos[0];
    pos[1]   = SHARED.enc_pos[1];
    vel[0]   = SHARED.enc_vel[0];
    vel[1]   = SHARED.enc_vel[1];
    imu_local = SHARED.imu;           // struct copy (64 B)
    estop_local  = SHARED.estop.load();
    calib_local  = SHARED.imu.calib;
  portEXIT_CRITICAL(&g_shared_mux);


  const uint32_t ms = millis();

  // joint_state msg 
  js_msg.header.stamp.sec     = ms / 1000;
  js_msg.header.stamp.nanosec = (ms % 1000) * 1000000;
  js_msg.position.data[0] = pos[0];
  js_msg.position.data[1] = pos[1];
  js_msg.velocity.data[0] = vel[0];
  js_msg.velocity.data[1] = vel[1];
  RCSOFT(rcl_publish(&pub_joint, &js_msg, nullptr));

  // imu msg
  imu_msg.header = js_msg.header; 
  imu_msg.orientation.w = imu_local.quat[0];
  imu_msg.orientation.x = imu_local.quat[1];
  imu_msg.orientation.y = imu_local.quat[2];
  imu_msg.orientation.z = imu_local.quat[3];

  imu_msg.angular_velocity.x    = imu_local.gyro[0];
  imu_msg.angular_velocity.y    = imu_local.gyro[1];
  imu_msg.angular_velocity.z    = imu_local.gyro[2];

  imu_msg.linear_acceleration.x = imu_local.accel[0];
  imu_msg.linear_acceleration.y = imu_local.accel[1];
  imu_msg.linear_acceleration.z = imu_local.accel[2];

  RCSOFT(rcl_publish(&pub_imu, &imu_msg, nullptr));

  // estop & calib msgs
  estop_msg.data = estop_local;
  calib_msg.data = calib_local;
  RCSOFT(rcl_publish(&pub_estop, &estop_msg, nullptr));
  RCSOFT(rcl_publish(&pub_calib, &calib_msg, nullptr));

  // spin executor (non-blocking)
  rclc_executor_spin_some(&executor, 0);

  delay(5); // 200 Hz
}