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
  memset(vel_buf_, 0, sizeof(vel_buf_));
	is_stop_ = false;
	iter_ = 0;
			
	goal_.header.frame_id = "map";
	goal_.pose.position.x = 0.0;
	goal_.pose.position.y = 0.0;
	goal_.pose.position.z = 0.0;
	goal_.pose.orientation.x = 0.0;
	goal_.pose.orientation.y = 0.0;
	goal_.pose.orientation.z = 0.0;
	goal_.pose.orientation.w = 1.0;
		
	geometry_msgs::Point point;
	// point.x = 2.5;
	// point.y = 0.0;
	// spin_points_.push_back(point);
	point.x = 0.0;
	point.y = 0.0;
	spin_points_.push_back(point);
	// point.x = 3.3;
	// point.y = -2.5;
	// spin_points_.push_back(point);
}

void StateProcess::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &StateProcess::odometryHandler, this);
  this->sub_global_path_ = nh.subscribe<nav_msgs::Path>("/global_planner/planner/plan", 5, &StateProcess::pathHandler, this);
  this->sub_vel = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, &StateProcess::velHandler, this);
  this->nav_goal_server = nh.advertiseService("/nav_goal", &StateProcess::navGoalHandler, this);
  this->nav_stop_server = nh.advertiseService("/nav_stop", &StateProcess::navStopHandler, this);
 
  // ROS publisher initialization
  this->pub_waypoint_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  this->pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 5);
  this->pub_stop_ = nh.advertise<std_msgs::Bool>("/stop", 5);
  this->pub_quat_ = nh.advertise<geometry_msgs::QuaternionStamped>("/world_quat", 5);

  this->loop_timer_ = nh.createTimer(ros::Duration(0.02), &StateProcess::loop, this);
  this->send_vel_timer_ = nh.createTimer(ros::Duration(0.003), &StateProcess::sendVelCallback, this);
  // this->receive_timer_ = nh.createTimer(ros::Duration(0.002), &StateProcess::receiveCallback, this);
  this->receive_thread_ = std::thread(&StateProcess::receiveCallback, this);
}

void StateProcess::odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = *odom;
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

bool StateProcess::navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res)
{
  goal_ = req.pose;
  goal_.header.stamp = ros::Time::now();
  is_stop_ = false;

  res.success = true;
  return true;
}

bool StateProcess::navStopHandler(sentry_srvs::NavStop::Request &req, sentry_srvs::NavStop::Response &res)
{
  is_stop_ = req.stop;

  res.success = true;
  return true;
}

void inline StateProcess::publishStop(const bool& flag)
{
  std_msgs::Bool msg;
  msg.data = flag;
  pub_stop_.publish(msg);
}

void StateProcess::sendVelCallback(const ros::TimerEvent& event)
{
  this->can_.send(CHASSIS_MODE_ID, vel_buf_, 7);
  // std::cout << "sent" << std::endl;
}

void StateProcess::transfer2Quaternion(u_char *buf, double *quaternion)
{
  int16_t q[4] = {0};
  memcpy(q, buf, 8);

  for (int i = 0; i < 4; i++)
  {
    quaternion[i] = (double)q[i] / 32768.0;
  }
  // double *q_;
  // q_ = quaternion;
  // std::cout<<"yaw"<<atan2f(2.0f*(q_[0]*q_[3]+q_[1]*q_[2]),2.0f*(q_[0]*q_[0]+q_[1]*q_[1]-1.0f))<<
  //             "pitch"<<asinf(-2.0f*(q_[1]*q_[3]-q_[0]*q_[2]))<<
  //             "roll"<<atan2f(2.0f*(q_[0]*q_[1]+q_[2]*q_[3]),2.0f*(q_[0]*q_[0]+q_[3]*q_[3])-1.0f)<<std::endl;
  // std::cout<<"q"<<q_[0]<<" "<<q_[1]<<" "<<q_[2]<<" "<<q_[3]<<std::endl;
}

void StateProcess::receiveCallback()
{
  while (ros::ok())
  {
    uint id = 0;
    u_char buf[8] = {0};
    u_char dlc = 0;

    double quaternion[4] = {0};
    Eigen::Quaterniond q_temp;
    static tf::TransformBroadcaster br;

    this->can_.receive(id, buf, dlc);

    switch (id)
    {
      case IMU_RECEIVE_ID:
      {
        this->transfer2Quaternion(buf, quaternion);
        q_temp = Eigen::Quaterniond(quaternion);
        q_temp.normalize();

        tf::Quaternion q(-q_temp.y(), -q_temp.z(), -q_temp.w(), q_temp.x());
        // tf::Quaternion q(q_temp.y(), -q_temp.z(), -q_temp.w(), -q_temp.x());

        geometry_msgs::QuaternionStamped msg;
        msg.header.stamp.fromSec(ros::Time::now().toSec());
        tf::quaternionTFToMsg(q, msg.quaternion);
        this->pub_quat_.publish(msg);

        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0, 0, 0));
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "world"));
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void StateProcess::loop(const ros::TimerEvent& event)
{
    publishStop(is_stop_);
    pub_goal_.publish(goal_);
    if (goal_.pose.position.x == 0 && goal_.pose.position.y == 0)
        // odom_.pose.pose.position.x < 1 && odom_.pose.pose.position.x > -1 &&
        // odom_.pose.pose.position.y < 1.5 && odom_.pose.pose.position.x > -1.5)
    {
      unsigned int n = iter_ % spin_points_.size();
      way_point_.header.frame_id = "map";
      way_point_.header.stamp = ros::Time::now();
      way_point_.point = spin_points_[n];
      if (ros::Time::now().toSec() - last_way_time_.toSec() > 4)
      {
        last_way_time_ = ros::Time::now();
        iter_++;
        if(iter_ > 1000000)
        {
          iter_ = 0;
        }
      }
    }
    pub_waypoint_.publish(way_point_);
}


} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_transporter");
  ros::NodeHandle nh;

  nav_transporter::StateProcess StateProcessObj;
  StateProcessObj.SubAndPubToROS(nh);

  ros::MultiThreadedSpinner s(3);
  ros::spin(s);

  return 0;
}