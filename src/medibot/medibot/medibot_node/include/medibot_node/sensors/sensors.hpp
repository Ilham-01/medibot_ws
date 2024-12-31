#ifndef MEDIBOT_NODE__SENSORS__SENSORS_HPP_
#define MEDIBOT_NODE__SENSORS__SENSORS_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "medibot_node/control_table.hpp"
#include "medibot_node/dynamixel_sdk_wrapper.hpp"

namespace robotis
{
namespace medibot
{
extern const ControlTable extern_control_table;
namespace sensors
{
class Sensors
{
public:
  explicit Sensors(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & frame_id = "")
  : nh_(nh),
    frame_id_(frame_id)
  {
  }

  virtual void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};
}  // namespace sensors
}  // namespace medibot
}  // namespace robotis
#endif  // MEDIBOT_NODE__SENSORS__SENSORS_HPP_
