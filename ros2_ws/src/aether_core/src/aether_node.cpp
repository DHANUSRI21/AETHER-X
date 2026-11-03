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

class AetherNode : public rclcpp::Node
{
public:
  AetherNode()
  : Node("aether_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "aether/system_status", 10);

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&AetherNode::publish_status, this));

    RCLCPP_INFO(this->get_logger(), "AETHER-X core node initialized");
  }

private:
  void publish_status()
  {
    std_msgs::msg::String message;
    message.data = "AETHER-X CORE ONLINE";

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AetherNode>());
  rclcpp::shutdown();

  return 0;
}
