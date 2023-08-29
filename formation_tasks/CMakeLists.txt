cmake_minimum_required(VERSION 3.8)
project(formation_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(task_math   REQUIRED)
find_package(rdyn_core  REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

list(APPEND THIS_PACKAGE_LIBRARIES_DEPENDENCIES
  task_math
  rdyn_core)

list(APPEND ${PROJECT_NAME}_ADD_HPP
  include/${PROJECT_NAME}/formation_tasks.hpp)
list(APPEND ${PROJECT_NAME}_ADD_CPP
  src/${PROJECT_NAME}/formation_tasks.cpp)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_ADD_CPP}
  ${${PROJECT_NAME}_ADD_HPP})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${THIS_PACKAGE_LIBRARIES_DEPENDENCIES})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

export(
  EXPORT ${PROJECT_NAME}Targets
  FILE "${PROJECT_BINARY_DIR}/follower_mpcTargets.cmake")

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "share/${PROJECT_NAME}/cmake")

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
