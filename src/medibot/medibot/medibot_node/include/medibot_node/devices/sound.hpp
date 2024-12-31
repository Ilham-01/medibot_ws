#ifndef MEDIBOT_NODE__DEVICES__SOUND_HPP_
#define MEDIBOT_NODE__DEVICES__SOUND_HPP_

#include <medibot_msgs/srv/sound.hpp>
#include <memory>
#include <string>
#include "medibot_node/devices/devices.hpp"

namespace robotis
{
namespace medibot
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<medibot_msgs::srv::Sound>::SharedPtr client,
    medibot_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<medibot_msgs::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace medibot
}  // namespace robotis
#endif  // MEDIBOT_NODE__DEVICES__SOUND_HPP_
