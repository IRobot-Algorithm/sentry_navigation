#include <ros/ros.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "state_processing.hpp"

namespace nav_transporter {

bool StateProcess::LoadParams(ros::NodeHandle &nh)
{

  nh.param<std::string>("config_path", config_path_, "./src/nav_transporter/config/params.yaml");
  cv::FileStorage fs(config_path_, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
     ROS_ERROR("Open nav transporter params fail!");
      exit(1);
  }

  cv::FileNode params = fs["params"];
  cv::FileNodeIterator it = params.begin(), it_end = params.end();

  //iterator through a sequence using FileNodeIterator
  for (; it != it_end; ++it)
  {
    StateParam param;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = (*it)["x"];
    goal.pose.position.y = (*it)["y"];
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, (*it)["pitch"], (*it)["yaw"]);
    
    std::vector<float> area;
    (*it)["area"] >> area;
    for (unsigned int i = 0; i < area.size()-1; i+=2)
    {
      cv::Point2d point;
      point.x = area[i];
      point.y = area[i+1];
      param.area.push_back(std::move(point));
    }

    std::vector<float> points;
    (*it)["points"] >> points;
    for (unsigned int i = 0; i < points.size()-1; i+=2)
    {
      cv::Point2d point;
      point.x = points[i];
      point.y = points[i+1];
      param.points.push_back(std::move(point));
    }

    params_.push_back(std::move(param));
  }

  fs.release();

  return true;
}

void StateProcess::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &StateProcess::odometryHandler, this);
  this->sub_global_path_ = nh.subscribe<nav_msgs::Path>("/global_planner/planner/plan", 5, &StateProcess::pathHandler, this);
  this->sub_vel = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, &StateProcess::velHandler, this);
  this->nav_state_server = nh.advertiseService("/state", &StateProcess::navStateHandler, this);

 
  // ROS publisher initialization
  this->pub_waypoint_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  this->pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 5);
  this->pub_stop_ = nh.advertise<std_msgs::Bool>("/stop", 5);

  this->send_vel_timer_ = nh.createTimer(ros::Duration(0.005), &StateProcess::sendVelCallback, this);
  this->loop_timer_ = nh.createTimer(ros::Duration(0.02), &StateProcess::loop, this);
}

void StateProcess::odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = *odom;
  pose_.x = float(odom_.pose.pose.position.x);
  pose_.y = float(odom_.pose.pose.position.y);
}

void StateProcess::pathHandler(const nav_msgs::Path::ConstPtr& path)
{
  unsigned int PathLength;
	PathLength = path->poses.size();
	if (PathLength != 0)
	{
    std::vector<geometry_msgs::PoseStamped>::const_iterator it = path->poses.begin();

    if (PathLength > 51)//81
    {
      it = path->poses.begin() + 49;//79
    }
    else
    {
      it = path->poses.end() - 1;
    }

    way_point_.point.x = it->pose.position.x;
    way_point_.point.y = it->pose.position.y;
    way_point_.point.z = it->pose.position.z;
    way_point_.header.stamp = ros::Time().now();
    way_point_.header.frame_id = "map";
	}
}

void StateProcess::velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
  int16_t velocity[3] = {0};

  velocity[0] = (int16_t)(vel->twist.linear.x * 1024);
  velocity[1] = (int16_t)(vel->twist.angular.z * 1024);
  velocity[2] = (int16_t)(vel->twist.linear.y * 1024);

  memcpy(&vel_buf_[1], velocity, 6);
}

bool StateProcess::navStateHandler(sentry_srvs::NavState::Request &req, sentry_srvs::NavState::Response &res)
{
  state_ = nav_transporter::State(req.state);
  is_stop_ = req.stop;
  should_stay_ = req.stay;
  goal_.pose = req.pose;
  goal_.header.stamp = ros::Time().now();
  goal_.header.frame_id = "map";
  if (in_area_)
    res.success = true;
  else
    res.success = false;

  return true;
}

void inline StateProcess::publishStop(const bool& flag)
{
  std_msgs::Bool msg;
  msg.data = flag;
  pub_stop_.publish(msg);
}

void StateProcess::autoNav(const StateParam& param, int& iter)
{  
  if (cv::pointPolygonTest(param.area, pose_, false) == 1) // 在内部
  {

    in_area_ = true;

    if (should_stay_)
    {
      int n = iter % int(param.points.size());
      geometry_msgs::PoseStamped way_point;
      way_point_.header.stamp = ros::Time::now();
      way_point_.header.frame_id = "map";
      way_point_.point.x = param.points[n].x;
      way_point_.point.y = param.points[n].y;

      if (ros::Time::now().toSec() - last_way_time_.toSec() > 4.0)
      {
        last_way_time_ = ros::Time::now();
        iter++;
        if (iter > 100000)
          iter = 0;
      } 

    }
    else
      pub_goal_.publish(goal_);

  }
  else
  {

    in_area_ = false;

    if (should_stay_)
      pub_goal_.publish(param.goal);
    else 
      pub_goal_.publish(goal_);
      
  }

}

void StateProcess::sendVelCallback(const ros::TimerEvent& event)
{
  this->can_.send(CHASSIS_MODE_ID, vel_buf_, 7);
}

void StateProcess::loop(const ros::TimerEvent& event)
{
    publishStop(is_stop_);
    pub_waypoint_.publish(way_point_);
    switch (state_)
    {
      case State::PATROL_AREA:
        // publishStop(false);
        autoNav(params_[static_cast<int32_t>(State::PATROL_AREA)], iter_);
        break;
      case State::SUPPLY_AREA: 
        // publishStop(false);
	
        break;
      case State::TRAPEZOIDAL_HEIGHTS: 
        // publishStop(false);
	
        break;
      case State::SMALL_RESOURCE_ISLAND: 
        // publishStop(false);
	
        break;
      case State::OUR_OUTPOST:
        // publishStop(false);
	
        break;
      case State::ENEMY_OUTPOST: 
        
        break;
    }
}


} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_transporter");
  ros::NodeHandle nh;

  nav_transporter::StateProcess StateProcessObj;
  StateProcessObj.SubAndPubToROS(nh);
  StateProcessObj.LoadParams(nh);

  ros::MultiThreadedSpinner s(2);
  ros::spin(s);

  return 0;
}