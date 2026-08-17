cmake_minimum_required(VERSION 3.8)
project(eod_av_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sync_check src/sync_check_node.cpp)
target_include_directories(sync_check PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(sync_check rclcpp)

install(TARGETS sync_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # Test puro de C++: sync_stats.hpp no depende de ROS, asi que no hace falta
  # gtest ni un middleware corriendo. Se registra con add_test, que es lo que
  # ejecuta `colcon test`.
  add_executable(test_sync_stats test/test_sync_stats.cpp)
  target_include_directories(test_sync_stats PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_sync_stats COMMAND test_sync_stats)
endif()

ament_package()
