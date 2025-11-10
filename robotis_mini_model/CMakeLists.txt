cmake_minimum_required(VERSION 3.28)
project(robotis_mini_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_program(XACRO_EXE xacro REQUIRED)

set(XACRO_IN ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robotis_mini.urdf.xacro)
set(URDF_OUT ${CMAKE_CURRENT_BINARY_DIR}/robotis_mini.urdf)

# Generate URDF at build
add_custom_command(
  OUTPUT ${URDF_OUT}
  COMMAND ${XACRO_EXE} --inorder ${XACRO_IN} -o ${URDF_OUT}
  DEPENDS ${XACRO_IN}
  COMMENT "Generating URDF from Xacro: ${XACRO_IN}"
  VERBATIM)

add_custom_target(generate_urdf ALL DEPENDS ${URDF_OUT})

# Install the generated URDF
install(FILES ${URDF_OUT} DESTINATION share/${PROJECT_NAME}/urdf)
# Install meshes.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

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
