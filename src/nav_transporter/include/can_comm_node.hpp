#ifndef CAN_COMM_NODE_HPP
#define CAN_COMM_NODE_HPP

#include <ros/ros.h>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include "can.hpp"

namespace nav_transporter {

class CanCommNode {
	public:
		CanCommNode();
		~CanCommNode() {};

  	void SubAndPubToROS(ros::NodeHandle &nh);

  	void LoadParams(ros::NodeHandle &nh);

	private:
		void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel);
		// void velHandler(const geometry_msgs::Twist::ConstPtr& vel);

		void sendVelCallback(const ros::TimerEvent& event);
		
		/*
		* @brief 电控imu四元数与odom差值
		* @auther wyq
		*/
    void syncPackages();

    void transfer2Quaternion(u_char *buf, double *quaterion);

    void receiveCallback();

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_vel;

		ros::Timer send_vel_timer_;

    geometry_msgs::Quaternion odom_quat_;
    double odom_time_;
    bool new_odom_;

    std::deque<geometry_msgs::QuaternionStamped::ConstPtr> quat_buffer_;
    double last_timestamp_quat;

    tf::Transform trans_;
    tf::Transform right_trans_;
    tf::Transform left_trans_;
		tf::TransformBroadcaster br_;

		u_char vel_buf_[7];
		transporter::Can can_;
    std::thread receive_thread_;

};


} // namespace nav_transporter

#endif