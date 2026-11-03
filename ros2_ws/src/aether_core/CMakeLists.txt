cmake_minimum_required(VERSION 3.16)
project(aether_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(aether_node
  src/aether_node.cpp
)

add_executable(aether_monitor
  src/aether_monitor.cpp
)

ament_target_dependencies(aether_node
  rclcpp
  std_msgs
)

ament_target_dependencies(aether_monitor
  rclcpp
  std_msgs
)

install(TARGETS
  aether_node
  aether_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_system_state_manager
    test/test_system_state_manager.cpp
  )

  target_include_directories(test_system_state_manager PRIVATE
    include
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
