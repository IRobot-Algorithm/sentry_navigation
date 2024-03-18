#ifndef CAN_COMM_NODE_HPP
#define CAN_COMM_NODE_HPP

#include <ros/ros.h>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include "usb.hpp"
#include "sentry_msgs/RefereeInformation.h"

namespace nav_transporter {

class UsbCommNode {
	public:
		UsbCommNode();
		~UsbCommNode();

  	void SubAndPubToROS(ros::NodeHandle &nh);

  	void LoadParams(ros::NodeHandle &nh);

	private:
		void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel);
		// void velHandler(const geometry_msgs::Twist::ConstPtr& vel);

		void sendVelCallback(const ros::TimerEvent& event);
		
		/*
		* @brief 电控imu四元数与odom差值
		*/
    void syncPackages();

    void receiveCallback();

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_vel_;
		ros::Publisher pub_referee_info_;

		ros::Timer send_vel_timer_;

    sentry_msgs::RefereeInformation referee_info_;
    geometry_msgs::Quaternion odom_quat_;
    double odom_time_;
    bool new_odom_;

    std::deque<geometry_msgs::QuaternionStamped::ConstPtr> quat_buffer_;
    double last_timestamp_quat;

    tf::Transform trans_;
    tf::Transform right_trans_;
    tf::Transform left_trans_;
		tf::TransformBroadcaster br_;

    int interface_usb_vid_;
    int interface_usb_pid_;
    int interface_usb_read_endpoint_;
    int interface_usb_write_endpoint_;
    int interface_usb_read_timeout_;
    int interface_usb_write_timeout_;

    transporter::NavVelocitySendPackage send_package_;

    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_; // usb通信接口
    std::thread receive_thread_;

};


} // namespace nav_transporter

#endif