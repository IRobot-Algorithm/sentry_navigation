//
// Created by Wu on 24-3-14.
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <Eigen/Dense>

#include "usb_comm_node.hpp"

using namespace std::chrono;

namespace nav_transporter
{
UsbCommNode::UsbCommNode()
{
  referee_info_.enemy_hp.resize(8);
}

UsbCommNode::~UsbCommNode()
{
    if (receive_thread_.joinable())
        receive_thread_.join();
}

void UsbCommNode::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &UsbCommNode::odomHandler, this);
  this->sub_vel_ = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, &UsbCommNode::velHandler, this);
  // this->sub_vel_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, &UsbCommNode::velHandler, this);
        
  this->pub_referee_info_ = nh.advertise<sentry_msgs::RefereeInformation>("/referee_info", 5);

  // ROS timer initialization
  // this->send_vel_timer_ = nh.createTimer(ros::Duration(0.005), &UsbCommNode::sendVelCallback, this);
  this->receive_thread_ = std::thread(&UsbCommNode::receiveCallback, this);
}

void UsbCommNode::LoadParams(ros::NodeHandle &nh)
{

  nh.param<int>("usb_comm/interface_usb_vid", interface_usb_vid_, 0x0483);
  nh.param<int>("usb_comm/interface_usb_pid", interface_usb_pid_, 0x5740);
  nh.param<int>("usb_comm/interface_usb_read_endpoint", interface_usb_read_endpoint_, 0x81);
  nh.param<int>("usb_comm/interface_usb_write_endpoint", interface_usb_write_endpoint_, 0x01);
  nh.param<int>("usb_comm/interface_usb_read_timeout", interface_usb_read_timeout_, 1);
  nh.param<int>("usb_comm/interface_usb_write_timeout", interface_usb_write_timeout_, 1);

  ROS_INFO("Init Transporter");
  transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
      interface_usb_vid_, 
      interface_usb_pid_, 
      interface_usb_read_endpoint_, 
      interface_usb_write_endpoint_, 
      interface_usb_read_timeout_, 
      interface_usb_write_timeout_
  );

  ROS_INFO("Open Transporter");
  if (transporter_->open() == true)
  {
      ROS_INFO("Success");
  }
  else
  {
      ROS_INFO("FAILED!!!");
  }
  ROS_INFO("Finish Init");

  std::vector<double> right_gimbal_T{3, 0.0};  // lidar-imu translation
  std::vector<double> left_gimbal_T{3, 0.0};  // lidar-imu rotation

  nh.param<std::vector<double>>("right_gimbal/extrinsic_T", right_gimbal_T, std::vector<double>());
  nh.param<std::vector<double>>("left_gimbal/extrinsic_T", left_gimbal_T, std::vector<double>());

  trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  trans_.setOrigin(tf::Vector3(0, 0, 0));
  right_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  right_trans_.setOrigin(tf::Vector3(right_gimbal_T[0],
                                     right_gimbal_T[1],
                                     right_gimbal_T[2]));
  left_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  left_trans_.setOrigin(tf::Vector3(left_gimbal_T[0],
                                    left_gimbal_T[1],
                                    left_gimbal_T[2]));

}

void UsbCommNode::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_quat_ = odom->pose.pose.orientation;
  odom_time_ = odom->header.stamp.toSec();
  new_odom_ = true;
}

void UsbCommNode::velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel)
// void UsbCommNode::velHandler(const geometry_msgs::Twist::ConstPtr& vel)
{
  send_package_._SOF = 0x55;
  send_package_._EOF = 0xFF;
  send_package_.ID = NAV_VELOCITY_SEND_ID;
  send_package_.vx = vel->twist.linear.x;
  send_package_.vy = vel->twist.linear.y;
  send_package_.yaw_imu = vel->twist.angular.z;
  transporter_->write((unsigned char *)&send_package_, sizeof(transporter::NavVelocitySendPackage));
}

// void UsbCommNode::sendVelCallback(const ros::TimerEvent& event)
// {
//   this->can_.send(VEL_SEND_ID, vel_buf_, 7);
//   // std::cout << "sent" << std::endl;
// }

void UsbCommNode::syncPackages()
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
  br_.sendTransform(tf::StampedTransform(trans_, ros::Time::now(), "map_link", "world"));

  return;
}

void UsbCommNode::receiveCallback()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {

    uint8_t receive_package[64];
    int read_size = transporter_->read(receive_package, 64);
    // ROS_INFO("read_size: %d", read_size);

    switch (receive_package[1])
    {
      case NAV_IMU_RECEIVE_ID:
      {
        transporter::NavIMUReceivePackage package;
        memcpy(&package, receive_package, 
                sizeof(transporter::NavIMUReceivePackage));

        geometry_msgs::QuaternionStamped::Ptr msg(new geometry_msgs::QuaternionStamped());

        msg->header.stamp.fromSec(ros::Time::now().toSec());
        msg->quaternion.x = -(double)package.q1;
        msg->quaternion.y = -(double)package.q2;
        msg->quaternion.z = -(double)package.q3;
        msg->quaternion.w = (double)package.q0;

        quat_buffer_.emplace_back(msg);

        right_trans_.setRotation(tf::createQuaternionFromYaw(package.RightMotorAngle));
        left_trans_.setRotation(tf::createQuaternionFromYaw(package.LeftMotorAngle));

        br_.sendTransform(tf::StampedTransform(right_trans_, ros::Time::now(), "base_link", "right_gimbal"));
        br_.sendTransform(tf::StampedTransform(left_trans_, ros::Time::now(), "base_link", "left_gimbal"));

        break;
      }
      case DESICION_REFEREE_RECEIVE_ID:
      {
        transporter::DesicionRefereeReceivePackage package;
        memcpy(&package, receive_package, 
                  sizeof(transporter::DesicionRefereeReceivePackage));

        int progress = static_cast<int>(package.game_type_progress & 0xf0 >> 4);
        if (progress == 4)
          referee_info_.game_start = true;
        else
          referee_info_.game_start = false;

        referee_info_.gameover_time = package.game_stage_remain_time;
        referee_info_.robot_hp = package.remain_HP;
        referee_info_.max_hp = package.max_HP;
        referee_info_.bullets = package.projectile_allowance_17mm;
          
         if (package.robot_id < 10) // red
         {
           referee_info_.our_outpost_hp = package.red_outpose_HP;
           referee_info_.our_base_hp = package.red_base_HP;
           referee_info_.enemy_hp[7] = package.blue_outpose_HP;
           referee_info_.enemy_hp[0] = package.blue_base_HP;
         }
         else // blue
         {
           referee_info_.our_outpost_hp = package.blue_outpose_HP;
           referee_info_.our_base_hp = package.blue_base_HP;
           referee_info_.enemy_hp[7] = package.red_outpose_HP;
           referee_info_.enemy_hp[0] = package.red_base_HP;
         }
         referee_info_.base_shield = package.base_state;
         referee_info_.gold_coins = package.remaining_gold_coin;

        referee_info_.in_supply = (package.rfid_status & 0x2000) == 0x2000;

        referee_info_.enemy_hp[1] =  package.hero_remain_HP;
        referee_info_.enemy_hp[2] =  package.engineer_remain_HP;
        referee_info_.enemy_hp[3] =  package.infantry3_remain_HP;
        referee_info_.enemy_hp[4] =  package.infantry4_remain_HP;
        referee_info_.enemy_hp[5] =  package.infantry5_remain_HP;
        referee_info_.enemy_hp[6] =  package.sentry_remain_HP;

        // TODO: keyward force back
        pub_referee_info_.publish(referee_info_);
        break;
      }
    }
    syncPackages();
    rate.sleep();
  }
}

} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "usb_comm_node");
  ros::NodeHandle nh;

  nav_transporter::UsbCommNode UsbCommNodeObj;
  UsbCommNodeObj.LoadParams(nh);
  UsbCommNodeObj.SubAndPubToROS(nh);

  ros::spin();
  return 0;

}