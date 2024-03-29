cmake_minimum_required(VERSION 3.8)
project(formation_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rdyn_core REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/interpolation.cpp
  include/${PROJECT_NAME}/interpolation.hpp
  include/${PROJECT_NAME}/data_types.hpp
  include/${PROJECT_NAME}/utils.hpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  moveit_msgs
  tf2_eigen
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rdyn_core
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

##########
## Test ##
##########################################
add_executable(test_node_interpolator
  src/test/test_node_interpolator.cpp)
target_link_libraries(test_node_interpolator PUBLIC
  ${PROJECT_NAME})
ament_target_dependencies(test_node_interpolator PUBLIC
  moveit_msgs)
install(TARGETS test_node_interpolator
    DESTINATION lib/${PROJECT_NAME})
##########################################

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 moveit_msgs)

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
