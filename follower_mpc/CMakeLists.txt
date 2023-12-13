cmake_minimum_required(VERSION 3.8)
project(follower_mpc)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake                REQUIRED)
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_lifecycle           REQUIRED)
# ros2_control
find_package(controller_interface       REQUIRED)
find_package(control_toolbox            REQUIRED)
find_package(realtime_tools             REQUIRED)
# Parameters
find_package(generate_parameter_library REQUIRED)
# rcl interfaces
find_package(geometry_msgs              REQUIRED)
find_package(sensor_msgs                REQUIRED)
find_package(formation_msgs             REQUIRED)
find_package(std_msgs                   REQUIRED)
# tf2
find_package(tf2_ros                    REQUIRED)
find_package(tf2_eigen                  REQUIRED)
# other
find_package(formation_utils            REQUIRED)
# Non ROS
find_package(Eigen3                     REQUIRED)
find_package(task_math                  REQUIRED)
find_package(rdyn_core                  REQUIRED)

generate_parameter_library(follower_mpc_parameters # cmake target name for the parameter library
  config/parameters_definition.yaml # path to input yaml file
)
  
list(APPEND ${PROJECT_NAME}_ADD_HPP # Only for IDE project visualization
  include/${PROJECT_NAME}/follower_mpc.hpp
)

list(APPEND ${PROJECT_NAME}_ADD_CPP
  src/${PROJECT_NAME}/follower_mpc.cpp)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_ADD_CPP}
  ${${PROJECT_NAME}_ADD_HPP})
target_link_libraries(${PROJECT_NAME} PUBLIC
  follower_mpc_parameters)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  controller_interface
  control_toolbox
  realtime_tools
  geometry_msgs
  sensor_msgs
  formation_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  task_math
  rdyn_core
  formation_utils
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
pluginlib_export_plugin_description_file(controller_interface plugin.xml) # Plugin

install(TARGETS ${PROJECT_NAME} follower_mpc_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_NAME}_LIBRARIES)

ament_package()
