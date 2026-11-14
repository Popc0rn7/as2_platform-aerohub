// Copyright 2026 Air-Suck contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Air-Suck contributors nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pixhawk_platform.hpp"

class Harness : public as2_platform_pixhawk::PixhawkPlatform
{
public:
  explicit Harness(const rclcpp::NodeOptions & options)
  : PixhawkPlatform(options) {}

  void mode()
  {
    as2_msgs::msg::ControlMode m;
    m.control_mode = m.BODY_RATES;
    m.yaw_mode = m.YAW_SPEED;
    platform_info_msg_.current_control_mode = m;
    if (!ownSetPlatformControlMode(m)) {
      throw std::runtime_error("mode rejected");
    }
  }

  void pair(const geometry_msgs::msg::TwistStamped & rates, const as2_msgs::msg::Thrust & thrust)
  {
    command_twist_msg_ = rates;
    command_thrust_msg_ = thrust;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto share = ament_index_cpp::get_package_share_directory("as2_platform_pixhawk");
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__ns:=/ipc_pairing_test",
      "-p", "control_modes_file:=" + share + "/config/control_modes.yaml",
      "--params-file", share + "/config/platform_config_file.yaml",
      "-p", "fmu_prefix:=/ipc_pairing"});
  auto platform = std::make_shared<Harness>(options);
  platform->mode();

  geometry_msgs::msg::TwistStamped rate;
  as2_msgs::msg::Thrust thrust;
  rate.header.frame_id = thrust.header.frame_id = platform->getBaseFrameId();
  rate.header.stamp = thrust.header.stamp = platform->now();
  rate.twist.angular.x = 0.2;
  rate.twist.angular.y = -0.3;
  rate.twist.angular.z = 0.4;
  thrust.thrust = 7.5;
  platform->pair(rate, thrust);
  if (!platform->ownSendCommand()) {
    return 1;
  }

  // Rate arrives first: the older thrust must not be combined with it.
  auto next = rate;
  next.header.stamp = rclcpp::Time(rate.header.stamp) + rclcpp::Duration::from_seconds(0.001);
  platform->pair(next, thrust);
  if (platform->ownSendCommand()) {
    return 2;
  }

  // Thrust arrives first, same invariant in the other order.
  auto next_thrust = thrust;
  next_thrust.header.stamp = next.header.stamp;
  platform->pair(rate, next_thrust);
  if (platform->ownSendCommand()) {
    return 3;
  }

  auto bad = thrust;
  bad.thrust = std::numeric_limits<float>::quiet_NaN();
  platform->pair(rate, bad);
  if (platform->ownSendCommand()) {
    return 4;
  }

  auto stale = rate;
  auto stale_thrust = thrust;
  stale.header.stamp = stale_thrust.header.stamp = platform->now() - rclcpp::Duration::from_seconds(
    0.2);
  platform->pair(stale, stale_thrust);
  if (platform->ownSendCommand()) {
    return 5;
  }

  bad = thrust;
  bad.header.frame_id = "earth";
  platform->pair(rate, bad);
  if (platform->ownSendCommand()) {
    return 6;
  }

  // A new matched sample is accepted without arming or connecting to any PX4.
  rate.header.stamp = thrust.header.stamp = platform->now();
  platform->pair(rate, thrust);
  if (!platform->ownSendCommand()) {
    return 7;
  }

  std::cout << "PASS: actual platform accepts fresh pairs and rejects both mixed orders, "
            << "NaN, stale, wrong frame\n";
  rclcpp::shutdown();
  return 0;
}
