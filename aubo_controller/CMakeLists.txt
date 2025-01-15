cmake_minimum_required(VERSION 3.12)
project(aubo_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aubo_ros2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(aubo_controller src/util.cpp main/main.cpp main/driver_sdk_copy.cpp)

target_include_directories(aubo_controller
  PRIVATE
  include
)

target_link_libraries(aubo_controller
${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x64/libauborobotcontroller.so
  )

ament_target_dependencies(aubo_controller
  rclcpp
  control_msgs trajectory_msgs sensor_msgs aubo_ros2_common)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x64/

  DESTINATION lib/${PROJECT_NAME}/lib/linux_x64/
)

install(TARGETS
  aubo_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
