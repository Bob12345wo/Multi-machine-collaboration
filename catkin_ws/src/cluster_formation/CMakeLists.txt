cmake_minimum_required(VERSION 3.0.2)
project(cluster_formation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  cluster_msgs
  cluster_common
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/FormationParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cluster_formation
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    cluster_msgs
    cluster_common
    dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(leader_controller_node
  src/leader_controller.cpp
  src/leader_controller_node.cpp
)

target_link_libraries(leader_controller_node
  cluster_common
  ${catkin_LIBRARIES}
)

add_dependencies(leader_controller_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
