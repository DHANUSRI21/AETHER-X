// Copyright 2026 Dhanusri Veerappan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AetherMonitor : public rclcpp::Node
{
public:
  AetherMonitor()
  : Node("aether_monitor"),
    last_message_time_(std::chrono::steady_clock::now()),
    heartbeat_lost_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
            "aether/system_status",
            10,
      [this](const std_msgs::msg::String::SharedPtr message)
      {
        last_message_time_ = std::chrono::steady_clock::now();

        if (heartbeat_lost_) {
          RCLCPP_INFO(
                        this->get_logger(),
                        "Heartbeat restored. System communication recovered.");
          heartbeat_lost_ = false;
        }

        RCLCPP_INFO(
                    this->get_logger(),
                    "System status received: '%s'",
                    message->data.c_str());
            });

    monitoring_timer_ = this->create_wall_timer(
            500ms,
      [this]() {check_heartbeat();});

    RCLCPP_INFO(
            this->get_logger(),
            "AETHER-X heartbeat monitor initialized");
  }

private:
  void check_heartbeat()
  {
    const auto elapsed = std::chrono::steady_clock::now() -
      last_message_time_;

    if (elapsed > 3s && !heartbeat_lost_) {
      RCLCPP_ERROR(
                this->get_logger(),
                "HEARTBEAT LOST: No system status received for more than 3 seconds!");

      heartbeat_lost_ = true;
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;

  std::chrono::steady_clock::time_point last_message_time_;
  bool heartbeat_lost_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AetherMonitor>());
  rclcpp::shutdown();

  return 0;
}
