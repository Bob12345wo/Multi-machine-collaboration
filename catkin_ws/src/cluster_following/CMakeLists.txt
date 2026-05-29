cmake_minimum_required(VERSION 3.0.2)
project(cluster_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  cluster_msgs
  cluster_common
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/FollowerParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cluster_following
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cluster_msgs
    cluster_common
    dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(follower_controller_node
  src/follower_controller.cpp
  src/follower_controller_node.cpp
)

target_link_libraries(follower_controller_node
  cluster_common
  ${catkin_LIBRARIES}
)

add_dependencies(follower_controller_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(map_follower_controller_node
  src/map_follower_controller_node.cpp
)

target_link_libraries(map_follower_controller_node
  cluster_common
  ${catkin_LIBRARIES}
)

add_dependencies(map_follower_controller_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(cmd_safety_filter_node
  src/cmd_safety_filter_node.cpp
)

target_link_libraries(cmd_safety_filter_node
  cluster_common
  ${catkin_LIBRARIES}
)

add_dependencies(cmd_safety_filter_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
