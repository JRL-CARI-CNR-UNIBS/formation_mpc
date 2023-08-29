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
find_package(rclcpp_action              REQUIRED)
find_package(pluginlib                  REQUIRED)
find_package(geometry_msgs              REQUIRED)
find_package(task_math                  REQUIRED)
find_package(controller_interface       REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(follower_mpc_parameters # cmake target name for the parameter library
  config/parameters.yaml # path to input yaml file
)

set(THIS_PACKAGE_LIBRARIES_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  pluginlib
  geometry_msgs
  task_math
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
  ${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
pluginlib_export_plugin_description_file(controller_interface plugin.xml) # Plugin

install(TARGETS ${PROJECT_NAME} follower_mpc_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  )

export(
  EXPORT ${PROJECT_NAME}Targets
  FILE "${PROJECT_BINARY_DIR}/follower_mpcTargets.cmake")

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "share/${PROJECT_NAME}/cmake"
  FILE follower_mpcTargets.cmake)

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

ament_package()
