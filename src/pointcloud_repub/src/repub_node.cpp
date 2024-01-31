#include <ros/ros.h>
#include <iostream>

#include "pointcloud_repub.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_repub");
  ros::NodeHandle nh;

  pointcloud_repub::PointCloudProcess PointCloudProcessObj;
  PointCloudProcessObj.SubAndPubToROS(nh);

  ros::MultiThreadedSpinner s(2);
  ros::spin(s);
 	// ros::Rate  loop_rate(50);
  // while(ros::ok())
 	// {	

	// 	ros::spinOnce();
	// }

  return 0;
}