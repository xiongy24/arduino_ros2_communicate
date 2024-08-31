cmake_minimum_required(VERSION 3.8)
project(arduino_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)

add_executable(arduino_serial_node src/arduino_serial_node.cpp)
ament_target_dependencies(arduino_serial_node rclcpp std_msgs serial_driver)

install(TARGETS
  arduino_serial_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
