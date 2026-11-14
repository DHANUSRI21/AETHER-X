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
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ImuSimulator : public rclcpp::Node
{
public:
  ImuSimulator()
  : Node("imu_simulator"),
    random_engine_(42),
    acceleration_noise_(0.0, 0.02),
    angular_velocity_noise_(0.0, 0.001)
  {
    publish_rate_hz_ = declare_parameter<double>(
      "publish_rate_hz", 100.0);

    acceleration_noise_stddev_ = declare_parameter<double>(
      "acceleration_noise_stddev", 0.02);

    angular_velocity_noise_stddev_ = declare_parameter<double>(
      "angular_velocity_noise_stddev", 0.001);

    frame_id_ = declare_parameter<std::string>(
      "frame_id", "imu_link");

    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/diff_drive_controller/odom");

    odom_timeout_sec_ = declare_parameter<double>(
      "odom_timeout_sec", 0.5);

    const int64_t random_seed = declare_parameter<int64_t>(
      "random_seed", 42);

    validate_parameters(random_seed);

    random_engine_.seed(static_cast<std::uint64_t>(random_seed));

    acceleration_noise_.param(
      std::normal_distribution<double>::param_type(
        0.0, std::max(acceleration_noise_stddev_, 1e-12)));

    angular_velocity_noise_.param(
      std::normal_distribution<double>::param_type(
        0.0, std::max(angular_velocity_noise_stddev_, 1e-12)));

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/aether/sensors/imu",
      rclcpp::SensorDataQoS());

    // Match the differential-drive odometry publisher QoS.
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.reliable();
    odom_qos.transient_local();

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(
        &ImuSimulator::odometry_callback,
        this,
        std::placeholders::_1));

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

    timer_ = create_wall_timer(
      period,
      std::bind(&ImuSimulator::publish_imu, this));

    RCLCPP_INFO(
      get_logger(),
      "Motion-aware IMU initialized: %.1f Hz, odometry='%s'",
      publish_rate_hz_,
      odom_topic_.c_str());
  }

private:
  static constexpr double kGravity = 9.80665;

  void validate_parameters(int64_t random_seed) const
  {
    if (!std::isfinite(publish_rate_hz_) ||
      publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 1000.0)
    {
      throw std::invalid_argument(
        "publish_rate_hz must be between 0 and 1000 Hz");
    }

    if (!std::isfinite(acceleration_noise_stddev_) ||
      acceleration_noise_stddev_ < 0.0)
    {
      throw std::invalid_argument(
        "acceleration_noise_stddev must be non-negative");
    }

    if (!std::isfinite(angular_velocity_noise_stddev_) ||
      angular_velocity_noise_stddev_ < 0.0)
    {
      throw std::invalid_argument(
        "angular_velocity_noise_stddev must be non-negative");
    }

    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }

    if (odom_topic_.empty()) {
      throw std::invalid_argument("odom_topic must not be empty");
    }

    if (!std::isfinite(odom_timeout_sec_) ||
      odom_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
        "odom_timeout_sec must be positive");
    }

    if (random_seed < 0) {
      throw std::invalid_argument(
        "random_seed must be non-negative");
    }
  }

  double sample_noise(
    std::normal_distribution<double> & distribution,
    double standard_deviation)
  {
    if (standard_deviation == 0.0) {
      return 0.0;
    }

    return distribution(random_engine_);
  }

  void odometry_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const double linear_velocity =
      message->twist.twist.linear.x;

    const double angular_velocity =
      message->twist.twist.angular.z;

    if (!std::isfinite(linear_velocity) ||
      !std::isfinite(angular_velocity))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring odometry containing non-finite velocity");
      return;
    }

    const auto receipt_time =
      std::chrono::steady_clock::now();

    if (has_previous_velocity_) {
      const double dt =
        std::chrono::duration<double>(
        receipt_time - previous_odom_time_).count();

      if (dt > 1e-4 && dt < odom_timeout_sec_) {
        longitudinal_acceleration_ =
          (linear_velocity - previous_linear_velocity_) / dt;
      } else {
        longitudinal_acceleration_ = 0.0;
      }
    } else {
      longitudinal_acceleration_ = 0.0;
      has_previous_velocity_ = true;
    }

    // Planar rigid-body acceleration in the robot frame.
    lateral_acceleration_ =
      linear_velocity * angular_velocity;

    angular_velocity_z_ = angular_velocity;

    previous_linear_velocity_ = linear_velocity;
    previous_odom_time_ = receipt_time;
    last_odom_receipt_time_ = receipt_time;

    has_received_odom_ = true;
  }

  void publish_imu()
  {
    const auto current_time =
      std::chrono::steady_clock::now();

    const bool odometry_fresh =
      has_received_odom_ &&
      std::chrono::duration<double>(
      current_time - last_odom_receipt_time_).count() <=
      odom_timeout_sec_;

    double acceleration_x = 0.0;
    double acceleration_y = 0.0;
    double angular_z = 0.0;

    if (odometry_fresh) {
      acceleration_x = longitudinal_acceleration_;
      acceleration_y = lateral_acceleration_;
      angular_z = angular_velocity_z_;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Odometry unavailable or stale; using stationary IMU fallback");
    }

    sensor_msgs::msg::Imu message;

    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;

    // Orientation is not estimated by this planar simulator.
    message.orientation_covariance[0] = -1.0;

    // Angular velocity: planar robot motion plus sensor noise.
    message.angular_velocity.x = sample_noise(
      angular_velocity_noise_,
      angular_velocity_noise_stddev_);

    message.angular_velocity.y = sample_noise(
      angular_velocity_noise_,
      angular_velocity_noise_stddev_);

    message.angular_velocity.z = angular_z + sample_noise(
      angular_velocity_noise_,
      angular_velocity_noise_stddev_);

    // Specific force in the upright robot sensor frame.
    message.linear_acceleration.x = acceleration_x + sample_noise(
      acceleration_noise_,
      acceleration_noise_stddev_);

    message.linear_acceleration.y = acceleration_y + sample_noise(
      acceleration_noise_,
      acceleration_noise_stddev_);

    message.linear_acceleration.z = kGravity + sample_noise(
      acceleration_noise_,
      acceleration_noise_stddev_);

    const double gyro_variance =
      angular_velocity_noise_stddev_ *
      angular_velocity_noise_stddev_;

    const double acceleration_variance =
      acceleration_noise_stddev_ *
      acceleration_noise_stddev_;

    message.angular_velocity_covariance[0] = gyro_variance;
    message.angular_velocity_covariance[4] = gyro_variance;
    message.angular_velocity_covariance[8] = gyro_variance;

    message.linear_acceleration_covariance[0] =
      acceleration_variance;
    message.linear_acceleration_covariance[4] =
      acceleration_variance;
    message.linear_acceleration_covariance[8] =
      acceleration_variance;

    publisher_->publish(message);
  }

  double publish_rate_hz_{100.0};
  double acceleration_noise_stddev_{0.02};
  double angular_velocity_noise_stddev_{0.001};

  double odom_timeout_sec_{0.5};

  double previous_linear_velocity_{0.0};
  double longitudinal_acceleration_{0.0};
  double lateral_acceleration_{0.0};
  double angular_velocity_z_{0.0};

  std::string frame_id_{"imu_link"};
  std::string odom_topic_{"/diff_drive_controller/odom"};

  bool has_received_odom_{false};
  bool has_previous_velocity_{false};

  std::chrono::steady_clock::time_point previous_odom_time_{};
  std::chrono::steady_clock::time_point last_odom_receipt_time_{};

  std::mt19937_64 random_engine_;
  std::normal_distribution<double> acceleration_noise_;
  std::normal_distribution<double> angular_velocity_noise_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ImuSimulator>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("imu_simulator"),
      "IMU simulator failed: %s",
      exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
