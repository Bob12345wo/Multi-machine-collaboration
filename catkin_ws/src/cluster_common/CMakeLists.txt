cmake_minimum_required(VERSION 3.0.2)
project(cluster_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  cluster_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cluster_common
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    cluster_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cluster_common
  src/pid.cpp
  src/pose_utils.cpp
)

target_link_libraries(cluster_common
  ${catkin_LIBRARIES}
)

add_dependencies(cluster_common
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
