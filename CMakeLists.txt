cmake_minimum_required(VERSION 3.0.2)
project(apsrc_v2x_viz)

set(catkin_deps
    apsrc_msgs
    geometry_msgs
    roscpp
    roslib
    std_msgs
    visualization_msgs
    geographic_msgs
    geodesy
    pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${catkin_deps}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(apsrc_v2x_viz_node src/v2x_viz_node.cpp)
target_link_libraries(
  apsrc_v2x_viz_node 
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)