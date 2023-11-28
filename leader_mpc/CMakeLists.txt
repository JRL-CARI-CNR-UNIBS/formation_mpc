cmake_minimum_required(VERSION 3.8)
project(leader_mpc)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS-related packages
find_package(ament_cmake                REQUIRED)
## rclcpp
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_lifecycle           REQUIRED)
find_package(rclcpp_action              REQUIRED)
find_package(pluginlib                  REQUIRED)
## msgs, srv
find_package(geometry_msgs              REQUIRED)
find_package(trajectory_msgs            REQUIRED)
find_package(formation_msgs             REQUIRED)
find_package(sensor_msgs                REQUIRED)
find_package(map_msgs                   REQUIRED)
find_package(moveit_msgs                   REQUIRED)
## tf2
find_package(tf2_ros                    REQUIRED)
find_package(tf2_eigen                  REQUIRED)
## parameters
find_package(generate_parameter_library REQUIRED)

# Non-ROS libraries
find_package(Eigen3                     REQUIRED)
find_package(task_math                  REQUIRED)
find_package(rdyn_core                  REQUIRED)


generate_parameter_library(leader_mpc_parameters # cmake target name for the parameter library
  config/parameters_definition.yaml # path to input yaml file
)

set(THIS_PACKAGE_LIBRARIES_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
#  pluginlib
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  map_msgs
  formation_msgs
  moveit_msgs
  task_math
  tf2_ros
  tf2_eigen
  Eigen3
  )
  
list(APPEND ${PROJECT_NAME}_ADD_HPP # Only for IDE project visualization
  include/${PROJECT_NAME}/leader_mpc.hpp
)

list(APPEND ${PROJECT_NAME}_ADD_CPP
  src/${PROJECT_NAME}/leader_mpc.cpp)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_ADD_HPP}
  ${${PROJECT_NAME}_ADD_CPP})
target_link_libraries(${PROJECT_NAME} PUBLIC
  leader_mpc_parameters
  rdyn_core)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_ON)
endif()

install(TARGETS ${PROJECT_NAME} leader_mpc_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  )

#export(
#  EXPORT export_${PROJECT_NAME}
#  FILE "${PROJECT_BINARY_DIR}/leader_mpcTargets.cmake")

#install(EXPORT export_${PROJECT_NAME}
#  DESTINATION "share/${PROJECT_NAME}/cmake"
#  FILE leader_mpcTargets.cmake)

install(DIRECTORY config launch
  DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

##########
## Node ##
##########
add_executable(leader_mpc_node
  src/leader_mpc_node.cpp)
#ament_target_dependencies(leader_mpc_node
#  ${THIS_PACKAGE_LIBRARIES_DEPENDENCIES}
#  )
target_link_libraries(leader_mpc_node PRIVATE
  ${PROJECT_NAME}
  leader_mpc_parameters)
install(TARGETS leader_mpc_node
  DESTINATION lib/${PROJECT_NAME})
##########

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
ament_export_dependencies(${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})

ament_package()
