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
#include <string>

#include "aether_core/system_state_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AetherMonitor : public rclcpp::Node
{
public:
  AetherMonitor()
  : Node("aether_monitor"),
    last_message_time_(std::chrono::steady_clock::now())
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "aether/system_status",
      10,
      [this](const std_msgs::msg::String::SharedPtr message)
      {
        last_message_time_ = std::chrono::steady_clock::now();

        handle_heartbeat_received();

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
  void handle_heartbeat_received()
  {
    switch (state_manager_.get_state()) {
      case aether::SystemState::INITIALIZING:
        transition_state(aether::SystemState::OPERATIONAL);
        break;

      case aether::SystemState::COMMUNICATION_LOST:
        transition_state(aether::SystemState::RECOVERING);
        break;

      case aether::SystemState::RECOVERING:
        transition_state(aether::SystemState::OPERATIONAL);
        break;

      case aether::SystemState::OPERATIONAL:
      case aether::SystemState::DEGRADED:
        break;
    }
  }

  void check_heartbeat()
  {
    const auto elapsed = std::chrono::steady_clock::now() -
      last_message_time_;

    if (elapsed > 3s &&
      state_manager_.get_state() != aether::SystemState::COMMUNICATION_LOST)
    {
      transition_state(aether::SystemState::COMMUNICATION_LOST);

      RCLCPP_ERROR(
        this->get_logger(),
        "HEARTBEAT LOST: No system status received for more than 3 seconds!");
    }
  }

  void transition_state(aether::SystemState new_state)
  {
    const auto previous_state = state_manager_.get_state();

    if (state_manager_.transition_to(new_state)) {
      const std::string previous_name = aether::to_string(previous_state);
      const std::string new_name = aether::to_string(new_state);

      RCLCPP_INFO(
        this->get_logger(),
        "System state transition: %s -> %s",
        previous_name.c_str(),
        new_name.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;

  std::chrono::steady_clock::time_point last_message_time_;
  aether::SystemStateManager state_manager_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AetherMonitor>());
  rclcpp::shutdown();

  return 0;
}
