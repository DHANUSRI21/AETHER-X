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
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class GnssSimulator : public rclcpp::Node
{
public:
  GnssSimulator()
  : Node("gnss_simulator"),
    random_engine_(42),
    standard_normal_(0.0, 1.0)
  {
    latitude_deg_ = declare_parameter<double>("latitude_deg", 49.4431);
    longitude_deg_ = declare_parameter<double>("longitude_deg", 1.0993);
    altitude_m_ = declare_parameter<double>("altitude_m", 20.0);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    horizontal_noise_stddev_m_ =
      declare_parameter<double>("horizontal_noise_stddev_m", 1.5);

    vertical_noise_stddev_m_ =
      declare_parameter<double>("vertical_noise_stddev_m", 3.0);

    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");

    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/diff_drive_controller/odom");

    odom_timeout_sec_ =
      declare_parameter<double>("odom_timeout_sec", 0.5);

    antenna_offset_x_m_ =
      declare_parameter<double>("antenna_offset_x_m", -0.20);

    antenna_offset_y_m_ =
      declare_parameter<double>("antenna_offset_y_m", 0.0);

    antenna_offset_z_m_ =
      declare_parameter<double>("antenna_offset_z_m", 0.210);

    const int64_t random_seed =
      declare_parameter<int64_t>("random_seed", 42);

    validate_parameters(random_seed);

    random_engine_.seed(static_cast<std::uint64_t>(random_seed));

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/aether/sensors/gnss",
      rclcpp::SensorDataQoS());

    // Match the differential drive odometry publisher's QoS.
    const auto odom_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .transient_local();

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(
        &GnssSimulator::odom_callback,
        this,
        std::placeholders::_1));

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

    timer_ = create_wall_timer(
      period,
      std::bind(&GnssSimulator::publish_gnss, this));

    RCLCPP_INFO(
      get_logger(),
      "Motion-aware GNSS initialized: %.1f Hz, odometry='%s'",
      publish_rate_hz_,
      odom_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Antenna offset relative to base_footprint: "
      "x=%.3f m, y=%.3f m, z=%.3f m",
      antenna_offset_x_m_,
      antenna_offset_y_m_,
      antenna_offset_z_m_);
  }

private:
  static constexpr double kEarthRadiusMeters = 6378137.0;
  static constexpr double kPi = 3.14159265358979323846;

  void validate_parameters(int64_t random_seed) const
  {
    if (!std::isfinite(latitude_deg_) ||
      latitude_deg_ < -89.0 || latitude_deg_ > 89.0)
    {
      throw std::invalid_argument(
              "latitude_deg must be between -89 and 89 degrees");
    }

    if (!std::isfinite(longitude_deg_) ||
      longitude_deg_ < -180.0 || longitude_deg_ > 180.0)
    {
      throw std::invalid_argument(
              "longitude_deg must be between -180 and 180 degrees");
    }

    if (!std::isfinite(altitude_m_)) {
      throw std::invalid_argument("altitude_m must be finite");
    }

    if (!std::isfinite(publish_rate_hz_) ||
      publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 100.0)
    {
      throw std::invalid_argument(
              "publish_rate_hz must be between 0 and 100 Hz");
    }

    if (!std::isfinite(horizontal_noise_stddev_m_) ||
      horizontal_noise_stddev_m_ < 0.0 ||
      horizontal_noise_stddev_m_ > 1000.0)
    {
      throw std::invalid_argument(
              "horizontal noise must be between 0 and 1000 metres");
    }

    if (!std::isfinite(vertical_noise_stddev_m_) ||
      vertical_noise_stddev_m_ < 0.0 ||
      vertical_noise_stddev_m_ > 1000.0)
    {
      throw std::invalid_argument(
              "vertical noise must be between 0 and 1000 metres");
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

    if (!std::isfinite(antenna_offset_x_m_) ||
      !std::isfinite(antenna_offset_y_m_) ||
      !std::isfinite(antenna_offset_z_m_))
    {
      throw std::invalid_argument(
              "GNSS antenna offsets must be finite");
    }

    if (random_seed < 0) {
      throw std::invalid_argument("random_seed must be non-negative");
    }
  }

  double normalize_longitude(double longitude) const
  {
    double normalized = std::fmod(longitude + 180.0, 360.0);

    if (normalized < 0.0) {
      normalized += 360.0;
    }

    return normalized - 180.0;
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;

    if (!std::isfinite(position.x) ||
      !std::isfinite(position.y) ||
      !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry containing non-finite pose values");
      return;
    }

    // Normalize the quaternion before extracting yaw.
    const double q_norm = std::sqrt(
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w);

    if (q_norm < 1.0e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry containing an invalid orientation quaternion");
      return;
    }

    const double qx = orientation.x / q_norm;
    const double qy = orientation.y / q_norm;
    const double qz = orientation.z / q_norm;
    const double qw = orientation.w / q_norm;

    const double yaw = std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));

    // Rotate the antenna's mounting offset into the odometry frame.
    const double antenna_x =
      position.x +
      std::cos(yaw) * antenna_offset_x_m_ -
      std::sin(yaw) * antenna_offset_y_m_;

    const double antenna_y =
      position.y +
      std::sin(yaw) * antenna_offset_x_m_ +
      std::cos(yaw) * antenna_offset_y_m_;

    const double antenna_z = position.z + antenna_offset_z_m_;

    // Calibrate the starting antenna location as the GNSS origin.
    if (!has_reference_pose_) {
      reference_antenna_x_ = antenna_x;
      reference_antenna_y_ = antenna_y;
      reference_antenna_z_ = antenna_z;

      has_reference_pose_ = true;

      RCLCPP_INFO(
        get_logger(),
        "GNSS reference pose calibrated from first odometry sample");
    }

    // Displacement of the antenna relative to its starting position.
    east_displacement_m_ = antenna_x - reference_antenna_x_;
    north_displacement_m_ = antenna_y - reference_antenna_y_;
    vertical_displacement_m_ = antenna_z - reference_antenna_z_;

    last_odom_received_ = std::chrono::steady_clock::now();
    has_odometry_ = true;
  }

  void publish_gnss()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    const bool odometry_fresh =
      has_odometry_ &&
      (now_steady - last_odom_received_) <=
      std::chrono::duration<double>(odom_timeout_sec_);

    if (!odometry_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry unavailable or stale; freezing GNSS displacement "
        "and reporting NO_FIX");
    }

    const double east_noise =
      standard_normal_(random_engine_) * horizontal_noise_stddev_m_;

    const double north_noise =
      standard_normal_(random_engine_) * horizontal_noise_stddev_m_;

    const double vertical_noise =
      standard_normal_(random_engine_) * vertical_noise_stddev_m_;

    const double latitude_rad = latitude_deg_ * kPi / 180.0;

    const double east_position =
      east_displacement_m_ + east_noise;

    const double north_position =
      north_displacement_m_ + north_noise;

    const double vertical_position =
      vertical_displacement_m_ + vertical_noise;

    const double latitude_offset_rad =
      north_position / kEarthRadiusMeters;

    const double longitude_offset_rad =
      east_position /
      (kEarthRadiusMeters * std::cos(latitude_rad));

    sensor_msgs::msg::NavSatFix message;

    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;

    message.status.status = odometry_fresh ?
      sensor_msgs::msg::NavSatStatus::STATUS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    message.status.service =
      sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    message.latitude =
      latitude_deg_ + latitude_offset_rad * 180.0 / kPi;

    message.longitude = normalize_longitude(
      longitude_deg_ + longitude_offset_rad * 180.0 / kPi);

    message.altitude = altitude_m_ + vertical_position;

    const double horizontal_variance =
      horizontal_noise_stddev_m_ * horizontal_noise_stddev_m_;

    const double vertical_variance =
      vertical_noise_stddev_m_ * vertical_noise_stddev_m_;

    message.position_covariance[0] = horizontal_variance;
    message.position_covariance[4] = horizontal_variance;
    message.position_covariance[8] = vertical_variance;

    message.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    publisher_->publish(message);
  }

  double latitude_deg_{49.4431};
  double longitude_deg_{1.0993};
  double altitude_m_{20.0};

  double publish_rate_hz_{10.0};
  double horizontal_noise_stddev_m_{1.5};
  double vertical_noise_stddev_m_{3.0};

  double odom_timeout_sec_{0.5};

  double antenna_offset_x_m_{-0.20};
  double antenna_offset_y_m_{0.0};
  double antenna_offset_z_m_{0.210};

  double reference_antenna_x_{0.0};
  double reference_antenna_y_{0.0};
  double reference_antenna_z_{0.0};

  double east_displacement_m_{0.0};
  double north_displacement_m_{0.0};
  double vertical_displacement_m_{0.0};

  std::string frame_id_{"gnss_link"};
  std::string odom_topic_{"/diff_drive_controller/odom"};

  bool has_odometry_{false};
  bool has_reference_pose_{false};

  std::chrono::steady_clock::time_point last_odom_received_;

  std::mt19937_64 random_engine_;
  std::normal_distribution<double> standard_normal_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<GnssSimulator>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("gnss_simulator"),
      "GNSS simulator failed: %s",
      exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
