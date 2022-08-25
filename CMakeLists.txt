cmake_minimum_required(VERSION 3.8)
project(ld19_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find_package(irobot_events_executor REQUIRED)
# find_package(irobot_lock_free_events_queue REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCE_FILES
  src/cmd_interface_linux.cpp
  src/ld19_node.cpp
  src/lipkg.cpp
  src/tofbf.cpp
  src/main.cpp
)

add_executable(ld19_node ${SOURCE_FILES})
target_link_libraries(ld19_node udev)
ament_target_dependencies(ld19_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ld19_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
