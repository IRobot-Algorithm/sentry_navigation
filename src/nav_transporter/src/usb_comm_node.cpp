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

void printBinary(uint32_t num) {
  int numBits = sizeof(uint32_t) * 8; // 获取整数的位数
  for (int i = numBits - 1; i >= 0; --i) {
    // 通过位操作符获取每一位的值并输出
    std::cout << ((num >> i) & 1);
  }
  std::cout << std::endl;
}

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
  this->buy_bullets_server = nh.advertiseService("/buy_bullets", &UsbCommNode::buyBulletsHandler, this);

  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, &UsbCommNode::odomHandler, this);
  this->sub_vel_ = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 1, &UsbCommNode::velHandler, this);
  // this->sub_vel_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, &UsbCommNode::velHandler, this);
        
  this->pub_referee_info_ = nh.advertise<sentry_msgs::RefereeInformation>("/referee_info", 1);
  this->pub_color_info_ = nh.advertise<std_msgs::Bool>("/color_info", 10);

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

  nh.param<std::vector<double>>("extrinsic/right_gimbal/extrinsic_T", right_gimbal_T, std::vector<double>());
  nh.param<std::vector<double>>("extrinsic/left_gimbal/extrinsic_T", left_gimbal_T, std::vector<double>());

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

  // package init
  send_package_._SOF = 0x55;
  send_package_._EOF = 0xFF;
  send_package_.ID = NAV_VELOCITY_SEND_ID;
  send_package_.chassis_mode = 3; // KEEP
  send_package_.vx = 0.0;
  send_package_.vy = 0.0;
  send_package_.yaw_imu = 0.0;
  send_package_.direction = 0; // normal
  send_package_.capacitance = 0; // closed
  setBit(send_package_.sentry_cmd, 0);    // 0位1 确认复活
  clearBit(send_package_.sentry_cmd, 1);  // 1位0 不兑换复活
  setBitsRange(send_package_.sentry_cmd, 2, 12, 0);     // 2-12位0 兑换发弹量
  setBitsRange(send_package_.sentry_cmd, 13, 16, 0);    // 13-16位0 远程兑换发弹量
  setBitsRange(send_package_.sentry_cmd, 17, 20, 0);    // 17-20位0 远程兑换血量

  // referee infomation
  referee_info_.force_back = false;
  referee_info_.keep_patrol = false;
  referee_info_.normal_mode = 0; // 反前哨站

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
  if (vel->twist.linear.z < 1e-4) 
  {
    send_package_.chassis_mode = 0; // SLOW_ROTING
  }
  else if (vel->twist.linear.z < 1 + 1e-4)
  {
    send_package_.chassis_mode = 1; // RANDOM_ROTING
  }
  else if (vel->twist.linear.z < 2 + 1e-4)
  {
    send_package_.chassis_mode = 2; // QUICK_ROTING
  }
  else if (vel->twist.linear.z < 3 + 1e-4)
  {
    send_package_.chassis_mode = 3; // KEEP
  }
  if (in_supply_)
  {
    send_package_.chassis_mode = 3; // KEEP
  }

  send_package_.vx = static_cast<float>(vel->twist.linear.x);
  send_package_.vy = static_cast<float>(vel->twist.linear.y);
  send_package_.yaw_imu = static_cast<float>(vel->twist.angular.z);
  if (vel->twist.angular.x < 1e-3) // normal
    send_package_.direction = 0;
  else if (vel->twist.angular.x < 1 + 1e-3) // 逆时针
    send_package_.direction = 1;
  else // 顺时针
    send_package_.direction = 2;

  if (vel->twist.angular.y > 0.5) // open
    send_package_.capacitance = 1;
  else // closed
    send_package_.capacitance = 0;

  transporter_->write((unsigned char *)&send_package_, sizeof(transporter::NavVelocitySendPackage));
  
}

bool UsbCommNode::buyBulletsHandler(sentry_srvs::BuyBullets::Request &req, sentry_srvs::BuyBullets::Response &res)
{
  setBitsRange(send_package_.sentry_cmd, 2, 12, req.bullets);     // 2-12位 兑换发弹量
  // transporter_->write((unsigned char *)&send_package_, sizeof(transporter::NavVelocitySendPackage));

  res.success = true;
  return true;
}

void UsbCommNode::sendVelCallback(const ros::TimerEvent& event)
{
  transporter_->write((unsigned char *)&send_package_, sizeof(transporter::NavVelocitySendPackage));
}

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
  // ros::Rate rate(5000);
  while (ros::ok())
  {
    // rate.sleep();
    // ros::Time t = ros::Time::now();

    uint8_t receive_package[64];
    int read_size = transporter_->read(receive_package, 64);
    // ROS_INFO("read_size: %d", read_size);

    // if (read_size == -1)
    //   return;

    switch (receive_package[1])
    {
      case NAV_IMU_RECEIVE_ID:
      {
        transporter::NavIMUReceivePackage package;
        memcpy(&package, receive_package, 
                sizeof(transporter::NavIMUReceivePackage));

        geometry_msgs::QuaternionStamped::Ptr msg(new geometry_msgs::QuaternionStamped());

        msg->header.stamp.fromSec(ros::Time::now().toSec());
        msg->quaternion.x = -static_cast<double>(package.q1);
        msg->quaternion.y = -static_cast<double>(package.q2);
        msg->quaternion.z = -static_cast<double>(package.q3);
        msg->quaternion.w = static_cast<double>(package.q0);

        quat_buffer_.emplace_back(msg);
        if (quat_buffer_.size() > 1000)
          quat_buffer_.pop_front();

        right_trans_.setRotation(tf::createQuaternionFromYaw(package.RightMotorAngle));
        left_trans_.setRotation(tf::createQuaternionFromYaw(package.LeftMotorAngle));

        br_.sendTransform(tf::StampedTransform(right_trans_, ros::Time::now(), "base_link", "right_gimbal"));
        br_.sendTransform(tf::StampedTransform(left_trans_, ros::Time::now(), "base_link", "left_gimbal"));

        break;
      }
      case DESICION_REFEREE_RECEIVE_ID:
      {
        static std_msgs::Bool color_info;
        transporter::DesicionRefereeReceivePackage package;
        memcpy(&package, receive_package, 
                  sizeof(transporter::DesicionRefereeReceivePackage));

        int progress = static_cast<int>((package.game_type_progress & 0xf0) >> 4);
        if (progress == 4)
          referee_info_.game_start = true;
        else
          referee_info_.game_start = false;

        referee_info_.gameover_time = package.game_stage_remain_time;
        referee_info_.max_hp = package.max_HP;
        referee_info_.bullets = package.projectile_allowance_17mm;
        
        if (package.robot_id < 10) // red
        {
          referee_info_.robot_hp = package.red_sentry_remain_HP;
          referee_info_.our_outpost_hp = package.red_outpose_HP;
          referee_info_.our_base_hp = package.red_base_HP;
          referee_info_.enemy_hp[0] = package.blue_base_HP;
          referee_info_.enemy_hp[1] = package.blue_hero_remain_HP;
          referee_info_.enemy_hp[2] = package.blue_engineer_remain_HP;
          referee_info_.enemy_hp[3] = package.blue_infantry3_remain_HP;
          referee_info_.enemy_hp[4] = package.blue_infantry4_remain_HP;
          referee_info_.enemy_hp[5] = package.blue_infantry5_remain_HP;
          referee_info_.enemy_hp[6] = package.blue_sentry_remain_HP;
          referee_info_.enemy_hp[7] = package.blue_outpose_HP;
          color_info.data = false;
        }
        else // blue
        {
          referee_info_.robot_hp = package.blue_sentry_remain_HP;
          referee_info_.our_outpost_hp = package.blue_outpose_HP;
          referee_info_.our_base_hp = package.blue_base_HP;
          referee_info_.enemy_hp[0] = package.red_base_HP;
          referee_info_.enemy_hp[1] = package.red_hero_remain_HP;
          referee_info_.enemy_hp[2] = package.red_engineer_remain_HP;
          referee_info_.enemy_hp[3] = package.red_infantry3_remain_HP;
          referee_info_.enemy_hp[4] = package.red_infantry4_remain_HP;
          referee_info_.enemy_hp[5] = package.red_infantry5_remain_HP;
          referee_info_.enemy_hp[6] = package.red_sentry_remain_HP;
          referee_info_.enemy_hp[7] = package.red_outpose_HP;
          color_info.data = true;
        }
        referee_info_.base_shield = package.base_state;
        referee_info_.gold_coins = package.remaining_gold_coin;
        referee_info_.bought_bullets = package.bought_bullets;

        referee_info_.rfid_status = package.rfid_status;
        in_supply_ = getBit(referee_info_.rfid_status, 13);
        referee_info_.force_back = false;
        referee_info_.keep_patrol = false;
        referee_info_.normal_mode = package.mode;

        // TODO: keyward force back
        pub_referee_info_.publish(referee_info_);
        pub_color_info_.publish(color_info);
        break;
      }
    }

    syncPackages();
    // std::cout << "cost : " << (ros::Time::now().toSec() - t.toSec()) * 1000 << "ms" << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(10));

  }
}

  inline void UsbCommNode::setBit(uint32_t& data, int pos)
  {
    data |= (static_cast<uint32_t>(1) << pos);
  }

  inline void UsbCommNode::clearBit(uint32_t& data, int pos)
  {
    data &= (static_cast<uint32_t>(0) << pos);
  }

  inline bool UsbCommNode::getBit(const uint32_t& data, int pos)
  {
    return (data >> pos) & static_cast<uint32_t>(1);
  }

  void UsbCommNode::setBitsRange(uint32_t &data, int start, int end, uint32_t value)
  {
    uint32_t mask = ~((~static_cast<uint32_t>(0) << start) & ((static_cast<uint32_t>(1) << (end + 1)) - 1));
    data &= mask;
    
    value <<= start;
    data |= value;
    // printBinary(data);
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