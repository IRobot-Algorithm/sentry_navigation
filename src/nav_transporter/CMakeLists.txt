cmake_minimum_required(VERSION 2.8.3)
project(nav_transporter)

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  sentry_srvs
)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs message_runtime 
  INCLUDE_DIRS
)

add_executable(nav_transporter src/state_processing.cpp src/can.cpp)
target_link_libraries(nav_transporter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
