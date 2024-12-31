
#ifndef MEDIBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define MEDIBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "medibot_node/odometry.hpp"

namespace robotis
{
namespace medibot
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace medibot
}  // namespace robotis
#endif  // MEDIBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
