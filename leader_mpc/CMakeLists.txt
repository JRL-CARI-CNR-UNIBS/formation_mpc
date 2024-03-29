cmake_minimum_required(VERSION 3.8)
project(leader_mpc)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#if(CMAKE_BUILD_TYPE MATCHES "Debug")
#  add_compile_options(-fsanitize=address -fno-sanitize=alignment)
#endif()

set(THIS_PACKAGE_ROS_LIBRARIES_DEPENDENCIES
  ament_cmake
  ## rclcpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  ## ros2_control
  controller_interface
  hardware_interface
  realtime_tools
  control_toolbox
  ## interfaces (msg, srv, action)
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  map_msgs
  moveit_msgs
  formation_msgs
  ## tf2
  tf2_ros
  tf2_eigen
  ## utils
  formation_utils
  ## Misc
  Eigen3
  )

set(THIS_PACKAGE_NON_ROS_LIBRARIES_DEPENDENCIES
  rdyn_core
  task_math)

set(THIS_PACKAGE_LIBRARIES_DEPENDENCIES
  ${THIS_PACKAGE_ROS_LIBRARIES_DEPENDENCIES}
  ${THIS_PACKAGE_NON_ROS_LIBRARIES_DEPENDENCIES})

foreach(package IN ITEMS ${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})
  find_package(${package} REQUIRED)
endforeach()

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(leader_mpc_parameters # cmake target name for the parameter library
  config/parameters_definition.yaml # path to input yaml file
)
  
list(APPEND ${PROJECT_NAME}_ADD_HPP # Only for IDE project visualization
  include/${PROJECT_NAME}/leader_mpc.hpp
)

list(APPEND ${PROJECT_NAME}_ADD_CPP
  src/${PROJECT_NAME}/leader_mpc.cpp)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_ADD_HPP}
  ${${PROJECT_NAME}_ADD_CPP})
target_link_libraries(${PROJECT_NAME} PUBLIC
  leader_mpc_parameters
  task_math
  ${THIS_PACKAGE_NON_ROS_LIBRARIES_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_ROS_LIBRARIES_DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_ON)
endif()

pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)


install(TARGETS ${PROJECT_NAME} leader_mpc_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  )

install(DIRECTORY config launch
  DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})

ament_package()
