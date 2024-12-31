#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

//#include <medibotv4/SensorState.h>
#include "Config.h"
#include "Kinematics.h"
#include "Motor.h"
#include "LED.h"

// Motor and LED objects
Motor LH_motor(LH_D1, LH_D2, LH_D3, LH_ENA, LH_ENB);
Motor RH_motor(RH_D1, RH_D2, RH_D3, RH_ENA, RH_ENB);
Motor PAN_motor(PAN_D1, PAN_D2, PAN_EN);
Motor TILT_motor(TILT_D1, TILT_D2, TILT_EN);
LED LH_led(LED_R_LH, LED_G_LH, LED_B_LH);
LED RH_led(LED_R_RH, LED_G_RH, LED_B_RH);
Kinematics Robot(LH_motor, RH_motor, LH_led, RH_led);

// Function prototypes
void LH_ISRA();
void LH_ISRB();
void RH_ISRA();
void RH_ISRB();
void EMG_STOP();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void cmd_vel_callback(const geometry_msgs::Twist& twist);

// Micro-ROS variables
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t lwheel_ticks_pub;
rcl_publisher_t rwheel_ticks_pub;
rcl_publisher_t lwheel_pwm_pub;
rcl_publisher_t rwheel_pwm_pub;

rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;

// Message objects
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int16 lwheel_ticks_msg;
std_msgs__msg__Int16 rwheel_ticks_msg;
std_msgs__msg__Int16 lwheel_pwm_msg;
std_msgs__msg__Int16 rwheel_pwm_msg;

unsigned long lastCmdVelReceived = 0;
float linearX_vel = 0, angularZ_vel = 0;
const float MIN_VELOCITY = 0.0;
const float MAX_VELOCITY = (2*PI*WHEEL_RADIUS*MAX_RPM)/(60*GEAR_REDUCTION);//0.728485253 m/s
//----------------------------------------------------------------------------------//

void setup(){
  // Pin Initialization
  pinMode(SW_MODE, INPUT_PULLUP);
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(LSR1, INPUT_PULLUP);
  pinMode(LSR2, INPUT_PULLUP);
  pinMode(LSR3, INPUT_PULLUP);
  pinMode(CS_FWD, INPUT_PULLUP);
  pinMode(CS_RVR, INPUT_PULLUP);
  pinMode(CS_LFT, INPUT_PULLUP);
  pinMode(CS_RGT, INPUT_PULLUP);
  pinMode(CS_STT, INPUT_PULLUP);
  pinMode(CS_STP, INPUT_PULLUP);
  pinMode(LH_ENA, INPUT_PULLUP);
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(LH_ENA), LH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), LH_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), RH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), RH_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOP), EMG_STOP, CHANGE); // Activated interrupt

  // Micro-ROS Initialization
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create Node
  rclc_node_init_default(&node, "arduino_node", "", &support);

  // Publishers
  rclc_publisher_init_default(&lwheel_ticks_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "lwheel_ticks");
  rclc_publisher_init_default(&rwheel_ticks_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rwheel_ticks");

  // Subscriber
  rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);
}
void loop() {
  // Execute callbacks for micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Update ROS connection status
  Robot.isRosConnected = (rcl_publisher_is_valid(&lwheel_ticks_pub) && rcl_publisher_is_valid(&rwheel_ticks_pub));

  // REMOTE MODE
  if (digitalRead(SW_MODE) && !digitalRead(ESTOP)) {
    // Stop the robot if not connected to ROS or no velocity command received after timeout
    if (!Robot.isRosConnected || (millis() - lastCmdVelReceived > CMD_VEL_TIMEOUT)) {
      linearX_vel = 0;
      angularZ_vel = 0;
      Robot.Move(MIN_PWM, MIN_PWM); // Stop motors
    }

    // Convert Linear X and Angular Z Velocity to PWM
    float left_vel = linearX_vel - angularZ_vel * (WHEEL_SEPARATION / 2);
    float right_vel = linearX_vel + angularZ_vel * (WHEEL_SEPARATION / 2);

    // Determine left and right direction using sign
    int left_dir = (left_vel > 0) ? 1 : -1;
    int right_dir = (right_vel > 0) ? 1 : -1;

    // Ensure calculated velocity is within the valid range
    if (fabs(left_vel) > MAX_VELOCITY) {
      left_vel = left_dir * MAX_VELOCITY;
    } else if (fabs(left_vel) < MIN_VELOCITY) {
      left_vel = left_dir * MIN_VELOCITY;
    }
    if (fabs(right_vel) > MAX_VELOCITY) {
      right_vel = right_dir * MAX_VELOCITY;
    } else if (fabs(right_vel) < MIN_VELOCITY) {
      right_vel = right_dir * MIN_VELOCITY;
    }

    // Map wheel velocity to PWM
    int left_pwm = round(mapFloat(fabs(left_vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
    int right_pwm = round(mapFloat(fabs(right_vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));

    // Actuate the motors
    lwheel_pwm_msg.data = left_dir * left_pwm;
    rwheel_pwm_msg.data = right_dir * right_pwm;
    Robot.Move(left_dir * left_pwm, right_dir * right_pwm);

    // Publish wheel PWM values
    if (RCL_RET_OK != rcl_publish(&lwheel_pwm_pub, &lwheel_pwm_msg, NULL)) {
      // Handle error for left wheel PWM publishing
    }
    if (RCL_RET_OK != rcl_publish(&rwheel_pwm_pub, &rwheel_pwm_msg, NULL)) {
      // Handle error for right wheel PWM publishing
    }
  }

  // RESCUE MODE
  if (!digitalRead(SW_MODE) && !digitalRead(ESTOP)) {
    // Read sensor states once
    bool CS_FWD_state = digitalRead(CS_FWD);
    bool CS_RVR_state = digitalRead(CS_RVR);
    bool CS_LFT_state = digitalRead(CS_LFT);
    bool CS_RGT_state = digitalRead(CS_RGT);
    bool CS_STT_state = digitalRead(CS_STT);
    bool CS_STP_state = digitalRead(CS_STP);

    // Forward
    if (!CS_FWD_state && CS_RVR_state && CS_LFT_state && CS_RGT_state && CS_STT_state && CS_STP_state) {
      Robot.Move(STRAIGHT_PWM, STRAIGHT_PWM);
    }
    // Reverse
    else if (CS_FWD_state && !CS_RVR_state && CS_LFT_state && CS_RGT_state && CS_STT_state && CS_STP_state) {
      Robot.Move(-STRAIGHT_PWM, -STRAIGHT_PWM);
    }
    // Turn Left
    else if (CS_FWD_state && CS_RVR_state && !CS_LFT_state && CS_RGT_state && CS_STT_state && CS_STP_state) {
      Robot.Move(-TURN_PWM, TURN_PWM);
    }
    // Turn Right
    else if (CS_FWD_state && CS_RVR_state && CS_LFT_state && !CS_RGT_state && CS_STT_state && CS_STP_state) {
      Robot.Move(TURN_PWM, -TURN_PWM);
    }
    // Pan Left
    else if (CS_FWD_state && CS_RVR_state && !CS_LFT_state && CS_RGT_state && !CS_STT_state && CS_STP_state) {
      PAN_motor.Rotate(1, PAN_LEFT_LIM, PAN_RIGHT_LIM);
    }
    // Pan Right
    else if (CS_FWD_state && CS_RVR_state && CS_LFT_state && !CS_RGT_state && !CS_STT_state && CS_STP_state) {
      PAN_motor.Rotate(-1, PAN_LEFT_LIM, PAN_RIGHT_LIM);
    }
    // Tilt Up
    else if (!CS_FWD_state && CS_RVR_state && CS_LFT_state && CS_RGT_state && CS_STT_state && !CS_STP_state) {
      TILT_motor.Rotate(1, TILT_UP_LIM, TILT_DOWN_LIM);
    }
    // Tilt Down
    else if (CS_FWD_state && !CS_RVR_state && CS_LFT_state && CS_RGT_state && CS_STT_state && !CS_STP_state) {
      TILT_motor.Rotate(-1, TILT_UP_LIM, TILT_DOWN_LIM);
    }
    // Default: Stop all motions
    else {
      Robot.Move(MIN_PWM, MIN_PWM);
      PAN_motor.Rotate(0);
      TILT_motor.Rotate(0);
    }
  }
  // EMERGENCY STOP
  else {
    Robot.isRosConnected = -1;  // Indicate disconnected state
    Robot.Move(DISABLE_PWM, DISABLE_PWM);  // Disable motor driver
    PAN_motor.Rotate(0);  // Stop pan
    TILT_motor.Rotate(0);  // Stop tilt
  }

  //Publishing data to ROS
  lwheel_ticks_pub.publish(&lwheel_ticks_msg);
  rwheel_ticks_pub.publish(&rwheel_ticks_msg);
  lwheel_pwm_pub.publish(&lwheel_pwm_msg);
  rwheel_pwm_pub.publish(&rwheel_pwm_msg);
  // sensor_state_msg.ir1 = analogRead(IR1);
  // sensor_state_msg.ir2 = analogRead(IR2);
  // sensor_state_msg.ir3 = analogRead(IR3);
  // sensor_state_msg.sonar = analogRead(SON1);
  // sensor_state_msg.csens = analogRead(CSENS);
  // sensor_state_msg.laser1 = digitalRead(LSR1);
  // sensor_state_msg.laser2 = digitalRead(LSR2);
  // sensor_state_msg.laser3 = digitalRead(LSR3);
  // sensor_state_msg.switch_mode = digitalRead(SW_MODE);
  // sensor_state_msg.estop = digitalRead(ESTOP);
  // sensor_state_msg.cs_stt = digitalRead(CS_STT);
  // sensor_state_msg.cs_stp = digitalRead(CS_STP);
  // sensor_state_msg.cs_fwd = digitalRead(CS_FWD);
  // sensor_state_msg.cs_rvr = digitalRead(CS_RVR);
  // sensor_state_msg.cs_lft = digitalRead(CS_LFT);
  // sensor_state_msg.cs_rgt = digitalRead(CS_RGT);
  // sensor_state_pub.publish(&sensor_state_msg);
  delay(200); //5Hz
}

////////////FUNCTION DEFINITIONS////////////////

void LH_ISRA() {
  // Handle encoder interrupt for left wheel
  lwheel_ticks_msg.data = LH_motor.doEncoderA();
}

void LH_ISRB() {
  // Handle encoder interrupt for left wheel
  lwheel_ticks_msg.data = LH_motor.doEncoderB();
}

void RH_ISRA() {
  // Handle encoder interrupt for right wheel
  rwheel_ticks_msg.data = RH_motor.doEncoderA();
}

void RH_ISRB() {
  // Handle encoder interrupt for right wheel
  rwheel_ticks_msg.data = RH_motor.doEncoderB();
}

void EMG_STOP() {
  // Emergency stop to halt all actuators
  Robot.isRosConnected = -1;
  Robot.Move(DISABLE_PWM, DISABLE_PWM); //Stop motors and disable motor driver
  PAN_motor.Rotate(0); //Stop pan
  TILT_motor.Rotate(0); //Stop tilt
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  // Scale and map a float value from one range to another
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_vel_callback(const geometry_msgs::Twist& twist) {
  // Update linear and angular velocity from ROS2 Twist message
  linearX_vel = twist.linear.x;
  angularZ_vel = twist.angular.z;
  lastCmdVelReceived = millis(); // Update timestamp for last command received
}




