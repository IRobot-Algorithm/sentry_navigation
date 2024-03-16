cmake_minimum_required(VERSION 3.0.2)
project(obstacle_generater)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS pcl_ros nav_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacleGenerating src/obstacleGenerating.cpp)
target_link_libraries(obstacleGenerating ${catkin_LIBRARIES})