#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "state_processing.hpp"

namespace nav_transporter {


StateProcess::StateProcess()
{
  exec_state_ = INIT;
			
	goal_.header.frame_id = "map";
	goal_.pose.position.x = 0.0;
	goal_.pose.position.y = 0.0;
	goal_.pose.position.z = 0.0;
	goal_.pose.orientation.x = 0.0;
	goal_.pose.orientation.y = 0.0;
	goal_.pose.orientation.z = 0.0;
	goal_.pose.orientation.w = 1.0;

  way_point_.header.frame_id = "map";
  way_point_.point.x = 0;
  way_point_.point.y = 0;
  way_point_.point.z = 0;
}

void StateProcess::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &StateProcess::odometryHandler, this);
  this->sub_global_path_ = nh.subscribe<nav_msgs::Path>("/global_planner/planner/plan", 5, &StateProcess::globalPathHandler, this);
  this->nav_goal_server = nh.advertiseService("/nav_goal", &StateProcess::navGoalHandler, this);
  this->nav_target_server = nh.advertiseService("/nav_target", &StateProcess::navTargetHandler, this);

  // ROS publisher initialization
  this->pub_waypoint_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  this->pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 5);

  this->loop_timer_ = nh.createTimer(ros::Duration(0.01), &StateProcess::loop, this);
}

void StateProcess::odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = *odom;
  have_odom_ = true;
}

void StateProcess::globalPathHandler(const nav_msgs::Path::ConstPtr& path)
{
  global_path_ = *path;
  path_init_ = true;
}

bool StateProcess::navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res)
{
  goal_.header.stamp = ros::Time::now();
  goal_ = req.pose;

  changeNavExecState(NAVIGATE, "desicion");

  if (sqrt((odom_.pose.pose.position.x - goal_.pose.position.x) * 
           (odom_.pose.pose.position.x - goal_.pose.position.x) + 
           (odom_.pose.pose.position.y - goal_.pose.position.y) * 
           (odom_.pose.pose.position.y - goal_.pose.position.y)) < 0.3)
    res.is_arrive = true;
  else
    res.is_arrive = false;

    res.is_arrive = true;
  res.odom.pose = odom_.pose.pose;
  return true;
}

bool StateProcess::navTargetHandler(sentry_srvs::NavTarget::Request &req, sentry_srvs::NavTarget::Response &res)
{
  static tf::TransformListener ls;
  tf::StampedTransform map2gimbal_transform, base2gimbal_transform, map2base_transform;
  if (req.gimbal) // 0 for right, 1 for left
  {
    try {
      ls.lookupTransform("/map", "/left_gimbal",  
                          req.pose.header.stamp, map2gimbal_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("TargetTrans : %s",ex.what());
      return true;
    }
    try {
      ls.lookupTransform("/base_link", "/left_gimbal",  
                          req.pose.header.stamp, base2gimbal_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("TargetTrans : %s",ex.what());
      return true;
    }
  }
  else
  {
    try {
      ls.lookupTransform("/map", "/right_gimbal",  
                          req.pose.header.stamp, map2gimbal_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("TargetTrans : %s",ex.what());
    }
    try {
      ls.lookupTransform("/base_link", "/right_gimbal",  
                          req.pose.header.stamp, base2gimbal_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("TargetTrans : %s",ex.what());
      return true;
    }
  }
  try {
    ls.lookupTransform("/map", "/base_link",  
                        req.pose.header.stamp, map2base_transform);
  }
  catch (tf::TransformException &ex) {
    ROS_WARN("TargetTrans : %s",ex.what());
    return true;
  }
  double yaw = tf::getYaw(map2base_transform.getRotation()) + tf::getYaw(base2gimbal_transform.getRotation());
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  way_point_.point.x = map2gimbal_transform.getOrigin().x() +
                       req.pose.pose.position.x * cos_yaw -
                       req.pose.pose.position.y * sin_yaw;
  way_point_.point.y = map2gimbal_transform.getOrigin().y() +
                       req.pose.pose.position.x * sin_yaw +
                       req.pose.pose.position.y * cos_yaw;

  if(req.is_lost)
    way_point_.point.z = 0;
  else
    way_point_.point.z = -10;

  changeNavExecState(TRACK, "desicion");

  res.success = true;
  return true;
}

void StateProcess::loop(const ros::TimerEvent& event)
{

  static int nav_num = 0;
  nav_num++;
  if (nav_num == 100)
  {
    printNavExecState();
    if (!have_odom_)
      ROS_WARN("no odom.");
    nav_num = 0;
  }

  switch (exec_state_)
  {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      break;
    }
    case TRACK:
    {
      pub_waypoint_.publish(way_point_);
      break;
    }
    case NAVIGATE:
    {
      pub_goal_.publish(goal_);
      if (path_init_)
        pub_waypoint_.publish(way_point_);
      // TODO: else ...
      break;
    }
  }

}

void StateProcess::changeNavExecState(NAV_EXEC_STATE new_state, std::string pos_call)
{
  static std::string state_str[3] = {"INIT", "TRACK", "NAVIGATE"};
  int pre_s = int(exec_state_);
  if (new_state == NAVIGATE && exec_state_ != NAVIGATE)
    path_init_ = false;
  exec_state_ = new_state;
  ROS_INFO("[ %s ]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void StateProcess::printNavExecState()
{
  static std::string state_str[3] = {"INIT", "TRACK", "NAVIGATE"};
  
  ROS_INFO("[NAV]: state: %s", state_str[int(exec_state_)].c_str());
}

void StateProcess::cutWaypointFromPath()
{
  unsigned int PathLength;
	PathLength = global_path_.poses.size();
	if (PathLength != 0)
	{
    std::vector<geometry_msgs::PoseStamped>::const_iterator it = global_path_.poses.begin();

    if (PathLength > 51)//81
    {
      it = global_path_.poses.begin() + 49;//79
    }
    else
    {
      it = global_path_.poses.end() - 1;
    }

    way_point_.point.x = it->pose.position.x;
    way_point_.point.y = it->pose.position.y;
    way_point_.point.z = 0;
    way_point_.header.stamp = ros::Time().now();
    way_point_.header.frame_id = "map";
	}
}


} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_processing");
  ros::NodeHandle nh;

  nav_transporter::StateProcess StateProcessObj;
  StateProcessObj.SubAndPubToROS(nh);

  ros::spin();

  return 0;
}