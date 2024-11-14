cmake_minimum_required(VERSION 3.8)
project(frontier_exp_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_client_cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Create library for FrontierHelper
add_library(frontier_helper src/frontier_helper.cpp)
target_include_directories(frontier_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/frontier_exp_cpp>)
target_compile_features(frontier_helper PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(frontier_helper 
  rclcpp 
  nav_msgs 
  geometry_msgs
  OpenCV)

add_executable(frontier_lc src/frontier_lc.cpp)
ament_target_dependencies(frontier_lc 
rclcpp 
rclcpp_lifecycle
nav2_util
nav_msgs 
geometry_msgs 
visualization_msgs
tf2 
tf2_ros 
tf2_geometry_msgs
std_srvs
OpenCV
nav_client_cpp)

# Link helper library to frontier_lc
target_link_libraries(frontier_lc frontier_helper)

target_include_directories(frontier_lc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/frontier_exp_cpp>)
target_compile_features(frontier_lc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS frontier_helper frontier_lc
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/
  DESTINATION include/)

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
