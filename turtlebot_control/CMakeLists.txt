cmake_minimum_required(VERSION 3.8)
project(turtlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(slam_toolbox)
# find_package(turtlelib) # MAYBE ADD THIS BACK IN WHEN NEEDED

# Intercept
add_executable(intercept src/intercept.cpp)
ament_target_dependencies(intercept rclcpp sensor_msgs visualization_msgs)
# target_link_libraries(intercept turtlelib::turtlelib "${cpp_typesupport_target}")

# Simple Move Obstacle Avoidance Node
add_executable(simple_move src/simple_move.cpp)
ament_target_dependencies(simple_move rclcpp sensor_msgs geometry_msgs)

# New node for choreographed movement
add_executable(choreographed src/choreographed.cpp)
ament_target_dependencies(choreographed rclcpp geometry_msgs)

# Add path publisher node
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
intercept
simple_move
choreographed
path_publisher
DESTINATION lib/${PROJECT_NAME})

target_compile_features(intercept PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(simple_move PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(choreographed PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(path_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


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

# Install launch files.
install(DIRECTORY
  config
  launch
  src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
