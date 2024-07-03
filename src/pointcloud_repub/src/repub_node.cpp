#include <ros/ros.h>
#include <iostream>

#include "pointcloud_repub.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_repub");
  ros::NodeHandle nh;

  pointcloud_repub::PointCloudProcess PointCloudProcessObj;
  PointCloudProcessObj.SubAndPubToROS(nh);
  PointCloudProcessObj.loadParams(nh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}