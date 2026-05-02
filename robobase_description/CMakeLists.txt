cmake_minimum_required(VERSION 3.8)
project(robobase_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install all xacro files in models and subdirectories
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING PATTERN "*.xacro" PATTERN "*.urdf"
)

install(
  DIRECTORY models/turtlebot3_house/
  DESTINATION share/${PROJECT_NAME}/models/turtlebot3_house
)

# Explicitly install all xacro files in models/bases
install(
  DIRECTORY models/bases/
  DESTINATION share/${PROJECT_NAME}/models/bases
  FILES_MATCHING PATTERN "*.xacro" PATTERN "*.urdf"
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