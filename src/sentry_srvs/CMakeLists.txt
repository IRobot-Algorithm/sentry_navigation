cmake_minimum_required(VERSION 2.8.3)
project(sentry_srvs)

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
  tf
  message_generation
)

add_service_files(
  FILES
  BuyBullets.srv
  NavGoal.srv
  NavTarget.srv
)

generate_messages( 
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
  INCLUDE_DIRS
)
