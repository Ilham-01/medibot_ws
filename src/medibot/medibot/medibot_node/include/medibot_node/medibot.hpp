#ifndef MEDIBOT_NODE__MEDIBOT_HPP_
#define MEDIBOT_NODE__MEDIBOT_HPP_

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <medibot_msgs/msg/sensor_state.hpp>

#include "medibot_node/control_table.hpp"
#include "medibot_node/devices/devices.hpp"
#include "medibot_node/devices/motor_power.hpp"
#include "medibot_node/devices/reset.hpp"
#include "medibot_node/devices/sound.hpp"
#include "medibot_node/dynamixel_sdk_wrapper.hpp"
#include "medibot_node/odometry.hpp"
#include "medibot_node/sensors/battery_state.hpp"
#include "medibot_node/sensors/imu.hpp"
#include "medibot_node/sensors/joint_state.hpp"
#include "medibot_node/sensors/sensor_state.hpp"
#include "medibot_node/sensors/sensors.hpp"

namespace robotis
{
namespace medibot
{
extern const ControlTable extern_control_table;
class MediBot : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit MediBot(const std::string & usb_port);
  virtual ~MediBot() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();
  

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors *> sensors_;
  std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace medibot
}  // namespace robotis
#endif  // MEDIBOT_NODE__MEDIBOT_HPP_
