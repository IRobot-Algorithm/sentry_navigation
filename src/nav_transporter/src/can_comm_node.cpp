#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "can_comm_node.hpp"

namespace nav_transporter {


CanCommNode::CanCommNode()
{
  memset(vel_buf_, 0, sizeof(vel_buf_));
  trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  trans_.setOrigin(tf::Vector3(0, 0, 0));
}

void CanCommNode::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &CanCommNode::odomHandler, this);
  this->sub_vel = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, &CanCommNode::velHandler, this);
 
  // ROS timer initialization
  this->send_vel_timer_ = nh.createTimer(ros::Duration(0.003), &CanCommNode::sendVelCallback, this);
  this->receive_thread_ = std::thread(&CanCommNode::receiveCallback, this);
}

void CanCommNode::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_quat_ = odom->pose.pose.orientation;
  odom_time_ = odom->header.stamp.toSec();
  new_odom_ = true;
}

void CanCommNode::velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
  int16_t velocity[3] = {0};

  velocity[0] = (int16_t)(vel->twist.linear.x * 1024);
  velocity[1] = (int16_t)(vel->twist.angular.z * 1024);
  velocity[2] = (int16_t)(vel->twist.linear.y * 1024);

  memcpy(&vel_buf_[1], velocity, 6);
}

void CanCommNode::sendVelCallback(const ros::TimerEvent& event)
{
  this->can_.send(CHASSIS_MODE_ID, vel_buf_, 7);
  // std::cout << "sent" << std::endl;
}


void CanCommNode::syncPackages()
{
  if (!new_odom_ || quat_buffer_.empty()) {
    return;
  }

  double odom_time = odom_time_;
  tf::Quaternion tf_quat = tf::Quaternion(odom_quat_.x, odom_quat_.y, odom_quat_.z, odom_quat_.w);

  geometry_msgs::QuaternionStamped::ConstPtr last_quat = quat_buffer_.front();
  geometry_msgs::QuaternionStamped::ConstPtr next_quat;
  double quat_time = quat_buffer_.front()->header.stamp.toSec();
  while ((!quat_buffer_.empty()) && (quat_time < odom_time))
  {
      quat_time = quat_buffer_.front()->header.stamp.toSec();
      if (quat_time > odom_time) 
      {
        next_quat = quat_buffer_.front();
        break;
      }
      last_quat = quat_buffer_.front();
      quat_buffer_.pop_front();
  }

  if (quat_buffer_.empty() || last_quat == nullptr || next_quat == nullptr)
    return;

 
  double t1 = last_quat->header.stamp.toSec();
  double t2 = next_quat->header.stamp.toSec();

  if (t2 - t1 > 0.01 || t2 - t1 < 0)
    return; // may be problem

  float radio = (odom_time - t1) / (t2 - t1);

  tf::Quaternion q1, q2;
  tf::quaternionMsgToTF(last_quat->quaternion, q1);
  tf::quaternionMsgToTF(next_quat->quaternion, q2);

  tf::Quaternion C_board_quat = q1.slerp(q2, radio) * tf_quat;
  C_board_quat.normalize();
  trans_.setRotation(C_board_quat);

  new_odom_ = false;
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(trans_, ros::Time::now(), "map_link", "world"));

  return;
}

void CanCommNode::transfer2Quaternion(u_char *buf, double *quaternion)
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

void CanCommNode::receiveCallback()
{
  while (ros::ok())
  {
    uint id = 0;
    u_char buf[8] = {0};
    u_char dlc = 0;

    double quaternion[4] = {0};
    Eigen::Quaterniond q_temp;

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

        geometry_msgs::QuaternionStamped::Ptr msg(new geometry_msgs::QuaternionStamped());

        msg->header.stamp.fromSec(ros::Time::now().toSec());
        msg->quaternion.x = -q_temp.y();
        msg->quaternion.y = -q_temp.z();
        msg->quaternion.z = -q_temp.w();
        msg->quaternion.w = q_temp.x();

        quat_buffer_.emplace_back(msg);
        break;
      }
    }
    syncPackages();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_comm_node");
  ros::NodeHandle nh;

  nav_transporter::CanCommNode CanCommNodeObj;
  CanCommNodeObj.SubAndPubToROS(nh);

  ros::spin();

  return 0;
}