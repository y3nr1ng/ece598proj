cmake_minimum_required(VERSION 3.20)
project(robotis_mini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

include_directories(
  include
)

# Hardware interface
add_library(hardware
  src/hardware.cpp
)
target_link_libraries(hardware
  rclcpp::rclcpp
  hardware_interface::hardware_interface
  controller_interface::controller_interface
)

add_executable(kinematics
  src/nodes/kinematics.cpp
)
target_link_libraries(kinematics
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

install(
  TARGETS
    hardware
    kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
