cmake_minimum_required(VERSION 3.16)
project(aether_sensors VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(imu_simulator
  src/imu_simulator.cpp
)

ament_target_dependencies(imu_simulator
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(gnss_simulator
  src/gnss_simulator.cpp
)

ament_target_dependencies(gnss_simulator
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(lidar_simulator
  src/lidar_simulator.cpp
)

ament_target_dependencies(lidar_simulator
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS
  imu_simulator
  gnss_simulator
  lidar_simulator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
