// Copyright 2026 Dhanusri Veerappan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

struct CircularObstacle
{
  double x;
  double y;
  double radius;
};

class LidarSimulator : public rclcpp::Node
{
public:
  LidarSimulator()
  : Node("lidar_simulator"),
    random_engine_(42),
    range_noise_(0.0, 0.01)
  {
    publish_rate_hz_ =
      declare_parameter<double>("publish_rate_hz", 10.0);

    range_min_ = declare_parameter<double>("range_min", 0.12);
    range_max_ = declare_parameter<double>("range_max", 30.0);

    angular_resolution_deg_ =
      declare_parameter<double>("angular_resolution_deg", 1.0);

    frame_id_ =
      declare_parameter<std::string>("frame_id", "lidar_link");

    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/diff_drive_controller/odom");

    odom_timeout_sec_ =
      declare_parameter<double>("odom_timeout_sec", 0.5);

    lidar_offset_x_m_ =
      declare_parameter<double>("lidar_offset_x_m", 0.15);

    lidar_offset_y_m_ =
      declare_parameter<double>("lidar_offset_y_m", 0.0);

    const int64_t random_seed =
      declare_parameter<int64_t>("random_seed", 42);

    validate_parameters(random_seed);

    random_engine_.seed(static_cast<std::uint64_t>(random_seed));

    beam_count_ = static_cast<std::size_t>(
      std::llround(360.0 / angular_resolution_deg_));

    angle_increment_ =
      2.0 * kPi / static_cast<double>(beam_count_);

    scan_time_ = 1.0 / publish_rate_hz_;

    range_noise_.param(
      std::normal_distribution<double>::param_type(0.0, 0.01));

    // Fixed obstacle coordinates in the odometry/world frame.
    obstacles_ = {
      {5.0, 0.0, 1.0},
      {3.5, 2.5, 0.6},
      {4.0, -3.0, 0.8},
      {8.0, 5.0, 1.2},
      {1.0, -6.0, 0.5},
      {-3.0, 0.0, 0.5}
    };

    // Runtime-controlled obstacle for dynamic navigation testing.
    declare_parameter<bool>("dynamic_obstacle_enabled", false);
    declare_parameter<double>("dynamic_obstacle_x", 4.8);
    declare_parameter<double>("dynamic_obstacle_y", -1.5);
    declare_parameter<double>("dynamic_obstacle_radius", 0.25);

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/aether/sensors/lidar",
      rclcpp::SensorDataQoS());

    // Match the differential-drive odometry publisher QoS.
    const auto odom_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .transient_local();

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(
        &LidarSimulator::odom_callback,
        this,
        std::placeholders::_1));

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(scan_time_));

    timer_ = create_wall_timer(
      period,
      std::bind(&LidarSimulator::publish_scan, this));

    RCLCPP_INFO(
      get_logger(),
      "Motion-aware LiDAR initialized: %.1f Hz, %zu beams, frame='%s'",
      publish_rate_hz_,
      beam_count_,
      frame_id_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "LiDAR odometry='%s', sensor offset=(%.3f, %.3f) m",
      odom_topic_.c_str(),
      lidar_offset_x_m_,
      lidar_offset_y_m_);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  void validate_parameters(int64_t random_seed) const
  {
    if (!std::isfinite(publish_rate_hz_) ||
      publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 50.0)
    {
      throw std::invalid_argument(
              "publish_rate_hz must be between 0 and 50 Hz");
    }

    if (!std::isfinite(range_min_) ||
      !std::isfinite(range_max_) ||
      range_min_ <= 0.0 || range_max_ <= range_min_)
    {
      throw std::invalid_argument("Invalid LiDAR range configuration");
    }

    if (!std::isfinite(angular_resolution_deg_) ||
      angular_resolution_deg_ < 0.25 ||
      angular_resolution_deg_ > 10.0)
    {
      throw std::invalid_argument(
              "Angular resolution must be between 0.25 and 10 degrees");
    }

    const double beam_ratio = 360.0 / angular_resolution_deg_;

    if (std::abs(beam_ratio - std::round(beam_ratio)) > 1e-9) {
      throw std::invalid_argument(
              "Angular resolution must divide 360 degrees exactly");
    }

    if (frame_id_.empty() || odom_topic_.empty()) {
      throw std::invalid_argument(
              "frame_id and odom_topic must not be empty");
    }

    if (!std::isfinite(odom_timeout_sec_) ||
      odom_timeout_sec_ <= 0.0 || odom_timeout_sec_ > 60.0)
    {
      throw std::invalid_argument(
              "odom_timeout_sec must be between 0 and 60 seconds");
    }

    if (!std::isfinite(lidar_offset_x_m_) ||
      !std::isfinite(lidar_offset_y_m_))
    {
      throw std::invalid_argument(
              "LiDAR mounting offsets must be finite");
    }

    if (random_seed < 0) {
      throw std::invalid_argument("random_seed must be non-negative");
    }
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;

    if (!std::isfinite(position.x) ||
      !std::isfinite(position.y) ||
      !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring non-finite odometry pose");
      return;
    }

    const double q_norm = std::sqrt(
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w);

    if (q_norm < 1.0e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid odometry quaternion");
      return;
    }

    const double qx = orientation.x / q_norm;
    const double qy = orientation.y / q_norm;
    const double qz = orientation.z / q_norm;
    const double qw = orientation.w / q_norm;

    robot_x_ = position.x;
    robot_y_ = position.y;

    robot_yaw_ = std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));

    last_odom_received_ = std::chrono::steady_clock::now();
    has_odometry_ = true;
  }

  double calculate_range(
    double angle,
    const CircularObstacle & obstacle) const
  {
    const double direction_x = std::cos(angle);
    const double direction_y = std::sin(angle);

    const double projection =
      obstacle.x * direction_x + obstacle.y * direction_y;

    const double center_distance_squared =
      obstacle.x * obstacle.x + obstacle.y * obstacle.y;

    const double discriminant =
      projection * projection -
      (center_distance_squared -
      obstacle.radius * obstacle.radius);

    if (discriminant < 0.0 || projection < 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    const double root = std::sqrt(discriminant);
    double distance = projection - root;

    if (distance < range_min_) {
      distance = projection + root;
    }

    if (distance < range_min_ || distance > range_max_) {
      return std::numeric_limits<double>::infinity();
    }

    return distance;
  }

  void publish_scan()
  {
    const bool odometry_fresh =
      has_odometry_ &&
      (std::chrono::steady_clock::now() - last_odom_received_) <=
      std::chrono::duration<double>(odom_timeout_sec_);

    if (!odometry_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry unavailable or stale; skipping LiDAR scan");
      return;
    }

    // Transform the LiDAR mounting position into the world frame.
    const double cos_yaw = std::cos(robot_yaw_);
    const double sin_yaw = std::sin(robot_yaw_);

    const double lidar_x =
      robot_x_ + cos_yaw * lidar_offset_x_m_ -
      sin_yaw * lidar_offset_y_m_;

    const double lidar_y =
      robot_y_ + sin_yaw * lidar_offset_x_m_ +
      cos_yaw * lidar_offset_y_m_;

    sensor_msgs::msg::LaserScan message;

    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;

    message.angle_min = static_cast<float>(-kPi);
    message.angle_increment = static_cast<float>(angle_increment_);

    message.angle_max = static_cast<float>(
      -kPi + (beam_count_ - 1) * angle_increment_);

    // All beams are computed from one robot-pose snapshot.
    message.time_increment = 0.0F;

    // Preserve the 10 Hz scan publication period.
    message.scan_time = static_cast<float>(scan_time_);
    message.range_min = static_cast<float>(range_min_);
    message.range_max = static_cast<float>(range_max_);

    message.ranges.resize(
      beam_count_,
      std::numeric_limits<float>::infinity());

    const bool dynamic_obstacle_enabled =
      get_parameter("dynamic_obstacle_enabled").as_bool();

    const CircularObstacle dynamic_obstacle{
      get_parameter("dynamic_obstacle_x").as_double(),
      get_parameter("dynamic_obstacle_y").as_double(),
      get_parameter("dynamic_obstacle_radius").as_double()
    };

    for (std::size_t i = 0; i < beam_count_; ++i) {
      const double angle =
        -kPi + static_cast<double>(i) * angle_increment_;

      double nearest_distance =
        std::numeric_limits<double>::infinity();

      const auto evaluate_obstacle =
        [&](const CircularObstacle & world_obstacle)
        {
          const double dx = world_obstacle.x - lidar_x;
          const double dy = world_obstacle.y - lidar_y;

          CircularObstacle local_obstacle{
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            world_obstacle.radius
          };

          const double distance =
            calculate_range(angle, local_obstacle);

          nearest_distance = std::min(nearest_distance, distance);
        };

      for (const auto & world_obstacle : obstacles_) {
        evaluate_obstacle(world_obstacle);
      }

      if (dynamic_obstacle_enabled) {
        evaluate_obstacle(dynamic_obstacle);
      }

      if (std::isfinite(nearest_distance)) {
        const double noisy_distance =
          nearest_distance + range_noise_(random_engine_);

        message.ranges[i] = static_cast<float>(
          std::clamp(noisy_distance, range_min_, range_max_));
      }
    }

    publisher_->publish(message);
  }

  double publish_rate_hz_{10.0};
  double range_min_{0.12};
  double range_max_{30.0};
  double angular_resolution_deg_{1.0};

  double angle_increment_{0.0};
  double scan_time_{0.0};

  double odom_timeout_sec_{0.5};
  double lidar_offset_x_m_{0.15};
  double lidar_offset_y_m_{0.0};

  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};

  std::size_t beam_count_{0};

  std::string frame_id_{"lidar_link"};
  std::string odom_topic_{"/diff_drive_controller/odom"};

  bool has_odometry_{false};
  std::chrono::steady_clock::time_point last_odom_received_;

  std::mt19937_64 random_engine_;
  std::normal_distribution<double> range_noise_;

  std::vector<CircularObstacle> obstacles_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<LidarSimulator>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lidar_simulator"),
      "LiDAR simulator failed: %s",
      exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
