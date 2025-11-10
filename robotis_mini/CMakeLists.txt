cmake_minimum_required(VERSION 3.28)
project(robotis_mini)

# Warnings for our code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_DYNAMIXEL "Build with DynamixelSDK dependency" ON)

find_package(ament_cmake REQUIRED)

# ----------------------------
# Interfaces (actions/services)
# ----------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(action_files
  action/RestoreInitialPose.action
)
set(srv_files
  srv/ComputeIK.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    sensor_msgs
    unique_identifier_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ----------------------------
# Core deps (targets)
# ----------------------------
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ros2_control / pluginlib for hardware
find_package(ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ----------------------------
# DynamixelSDK (compiled in-tree)
# ----------------------------
if(USE_DYNAMIXEL)
  file(GLOB_RECURSE dynamixel_sdk_files
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/DynamixelSDK/c++/src/dynamixel_sdk/*.cpp"
  )

  add_library(dynamixel_sdk STATIC
    ${dynamixel_sdk_files}
  )

  # -fPIC
  set_target_properties(dynamixel_sdk
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )

  # Treat SDK headers as SYSTEM to hide their warnings
  target_include_directories(dynamixel_sdk
    SYSTEM PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/DynamixelSDK/c++/include/dynamixel_sdk
  )
  target_compile_options(dynamixel_sdk
    PRIVATE
      $<$<CXX_COMPILER_ID:GNU>:-w>
      $<$<CXX_COMPILER_ID:Clang>:-w>
  )
endif()

# ----------------------------
# Hardware plugin (ros2_control SystemInterface)
# ----------------------------

# Simulate
add_library(hardware_sim
  SHARED
    src/hardware_impl.cpp
    src/hardware_sim.cpp
)
target_include_directories(hardware_sim
  PUBLIC
    include
)
target_link_libraries(hardware_sim
  rclcpp::rclcpp
  hardware_interface::hardware_interface
  controller_interface::controller_interface
  pluginlib::pluginlib
)

# Real
if(USE_DYNAMIXEL)
  add_library(hardware
    SHARED
      src/hardware_impl.cpp
      src/hardware.cpp
  )
  target_include_directories(hardware
    PUBLIC
      include
      lib/DynamixelSDK/c++/include
  )
  target_link_libraries(hardware
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    controller_interface::controller_interface
    pluginlib::pluginlib
    dynamixel_sdk
  )
endif()

# Export plugin XML for controller_manager to discover.
pluginlib_export_plugin_description_file(hardware_interface
  plugin_description/hardware.xml
)

# ----------------------------
# Kinematics node
# ----------------------------
add_executable(kinematics
  src/nodes/kinematics.cpp
  src/ik.cpp
)

target_include_directories(kinematics
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(kinematics
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  ${cpp_typesupport_target}
)

# ----------------------------
# Restore-initial-pose node (uses FollowJointTrajectory)
# ----------------------------
add_executable(restore_initial_pose
  src/nodes/restore_initial_pose.cpp
)

target_include_directories(restore_initial_pose
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(restore_initial_pose
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  control_msgs::control_msgs__rosidl_typesupport_cpp
  trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
  ${cpp_typesupport_target}
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS
    hardware_sim
    kinematics
    restore_initial_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

if(USE_DYNAMIXEL)
  install(
    TARGETS
      hardware
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()

install(
  DIRECTORY
    config
    plugin_description
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------
# Testing
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
