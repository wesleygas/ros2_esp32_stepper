#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include "FastAccelStepper.h"

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t accel_subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
geometry_msgs__msg__Twist sub_msg;
std_msgs__msg__Int32 accel_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

// As in StepperDemo for Motor 1 on ESP32

#define WIFI_SSID
#define WIFI_PWD 

#define meter_per_sec_to_steps_per_sec 1600.0
#define rad_per_sec_to_steps_per_sec 800.0
#define initial_acceleration 800

#define enablePinStepper 12

#define rdirPinStepper 16
#define rstepPinStepper 26

#define ldirPinStepper 27
#define lstepPinStepper 25


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *rstepper = NULL;
FastAccelStepper *lstepper = NULL;

int32_t global_acceleration = initial_acceleration;

#define LED_PIN 22

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void set_stepper_speed(FastAccelStepper *stepper, int32_t stepper_speed){
  stepper->setSpeedInHz(abs(stepper_speed));
  if(stepper_speed == 0){
    stepper->stopMove();
  }else if(stepper_speed > 0){
    stepper->moveByAcceleration(global_acceleration);
  }else if(stepper_speed < 0){
    stepper->moveByAcceleration(-global_acceleration);
  }
}

void twist_subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * sub_msg = (const geometry_msgs__msg__Twist *)msgin;
  // digitalWrite(LED_PIN, (sub_msg->linear.y == 0) ? LOW : HIGH);  
  //Serial.printf("Received: Linear %.02f | Angular %.2f\n", sub_msg->linear.x, sub_msg->angular.z);
  int32_t rstep_speed = sub_msg->linear.x*meter_per_sec_to_steps_per_sec + sub_msg->angular.z*rad_per_sec_to_steps_per_sec;
  int32_t lstep_speed = sub_msg->linear.x*meter_per_sec_to_steps_per_sec - sub_msg->angular.z*rad_per_sec_to_steps_per_sec;
  set_stepper_speed(rstepper,rstep_speed);
  set_stepper_speed(lstepper,-lstep_speed);
}

void accel_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * accel_msg = (const std_msgs__msg__Int32 *)msgin;
  //Serial.printf("Received acceleration: %d\n", accel_msg->data);
  global_acceleration = accel_msg->data;
}

void setup_ros_sub(){
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &accel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stepper_accel"));
  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &sub_msg, &twist_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &accel_subscriber, &accel_msg, &accel_subscription_callback, ON_NEW_DATA));

}

void setup_ros_pub(){
  //create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "step_pos"));
    msg.data = 0;
}

void setup_steppers(){
  // right stepper
  rstepper = engine.stepperConnectToPin(rstepPinStepper);
  if (rstepper) {
    rstepper->setDirectionPin(rdirPinStepper);
    rstepper->setEnablePin(enablePinStepper);
    rstepper->setAutoEnable(true);

    rstepper->setSpeedInHz(meter_per_sec_to_steps_per_sec);
    // rstepper->moveByAcceleration(initial_acceleration);
  }
  // left stepper
  lstepper = engine.stepperConnectToPin(lstepPinStepper);
  if (lstepper) {
    lstepper->setDirectionPin(ldirPinStepper);
    lstepper->setEnablePin(enablePinStepper);
    lstepper->setAutoEnable(true);

    lstepper->setSpeedInHz(meter_per_sec_to_steps_per_sec);
    // lstepper->moveByAcceleration(initial_acceleration);
  }
}

void setup() {
  //Serial.begin(115200);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PWD, "192.168.0.11", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "stepper32", "", &support));

  setup_ros_pub();
  setup_ros_sub();
  
  engine.init();
  setup_steppers();

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // Serial.printf("Message: %d\n", msg.data);
    msg.data++;
    if(msg.data > 60000) msg.data = 0;
}
