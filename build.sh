#! /bin/bash
set +e

source /opt/ros/noetic/setup.bash
./src/livox_ros_driver2/build.sh ROS1

source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2"
catkin_make -DCATKIN_WHITELIST_PACKAGES="sentry_srvs"
catkin_make -DCATKIN_WHITELIST_PACKAGES="sentry_msgs"
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES="fast_lio"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""