cmake_minimum_required(VERSION 3.8)
project(ars430_ros_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# radar_publisher gets file capabilities via setcap (cap_net_raw); the dynamic
# linker then runs in secure mode and IGNORES LD_LIBRARY_PATH, so every ROS
# library must be findable through absolute RPATHs baked into the binary.
# --disable-new-dtags writes classic DT_RPATH, which (unlike DT_RUNPATH) also
# covers the transitive dependencies of librclcpp and friends.
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_link_options("-Wl,--disable-new-dtags")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarDetection.msg"
  "msg/RadarPacket.msg"
  "msg/RadarObject.msg"
  "msg/RadarObjectList.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(radar_publisher src/radar_publisher_node.cpp)
ament_target_dependencies(radar_publisher rclcpp std_msgs)
target_link_libraries(radar_publisher "${cpp_typesupport_target}" pcap)

add_executable(radar_processor src/radar_processor_node.cpp)
ament_target_dependencies(radar_processor rclcpp std_msgs)
target_link_libraries(radar_processor "${cpp_typesupport_target}")

add_executable(radar_visualizer src/radar_visualizer_node.cpp)
ament_target_dependencies(radar_visualizer rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs tf2_ros)
target_link_libraries(radar_visualizer "${cpp_typesupport_target}")

add_executable(radar_objects src/radar_objects_node.cpp)
ament_target_dependencies(radar_objects rclcpp std_msgs visualization_msgs geometry_msgs)
target_link_libraries(radar_objects "${cpp_typesupport_target}")

add_executable(radar_clusters src/radar_clusters_node.cpp)
ament_target_dependencies(radar_clusters rclcpp std_msgs visualization_msgs geometry_msgs)
target_link_libraries(radar_clusters "${cpp_typesupport_target}")

install(TARGETS radar_publisher radar_processor radar_visualizer radar_objects radar_clusters
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/radar_stats.py
  scripts/decode_v2_pcap.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
