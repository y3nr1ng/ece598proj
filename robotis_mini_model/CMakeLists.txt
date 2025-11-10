cmake_minimum_required(VERSION 3.28)
project(robotis_mini_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robotis_mini.urdf.xacro
  TARGET robotis_mini
  INSTALL DESTINATION urdf
  REMAP use_sim:=true # use dynamixel or not
)

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

  # Exclude generated dirs so linters don't recurse into them.
  set(AMENT_LINT_AUTO_EXCLUDE
    "build/*;install/*;log/*;ament_cmake_index/*;**/ament_cmake_index/*")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
