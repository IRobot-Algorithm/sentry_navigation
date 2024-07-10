#ifndef CAN_COMM_NODE_HPP
#define CAN_COMM_NODE_HPP

#include <ros/ros.h>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include "usb.hpp"
#include "sentry_msgs/RefereeInformation.h"
#include "sentry_msgs/RecordInformation.h"
#include "sentry_srvs/BuyBullets.h"

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

		void vizPathHandler(const visualization_msgs::Marker::ConstPtr& path);

		/*
		* @brief 决策购买弹丸服务器
		* @auther wyq
		*/
		bool buyBulletsHandler(sentry_srvs::BuyBullets::Request &req, sentry_srvs::BuyBullets::Response &res);

		void sendVelCallback(const ros::TimerEvent& event);
		
		void sendRecordCallback(const ros::TimerEvent& event);

		/*
		* @brief 电控imu四元数与odom差值
		*/
    void syncPackages();

    void receiveCallback();

		/*
		* @brief uint32 pos位赋值1
		*/
    inline void setBit(uint32_t& data, int pos);

		/*
		* @brief uint32 pos位赋值0
		*/
    inline void clearBit(uint32_t& data, int pos);

		/*
		* @brief 获得 uint32 pos位
		*/
    inline bool getBit(const uint32_t& data, int pos);

		/*
		* @brief uint32 start到end位赋值为value
		*/
    void setBitsRange(uint32_t &data, int start, int end, uint32_t value);

		ros::ServiceServer buy_bullets_server;
		ros::Subscriber sub_odom_;
		ros::Subscriber sub_vel_;
		ros::Subscriber sub_path_;
		ros::Publisher pub_referee_info_;
		ros::Publisher pub_record_info_;
		ros::Publisher pub_color_info_;
		ros::Publisher pub_uwb_;

		ros::Timer send_vel_timer_;
		ros::Timer send_record_timer_;

    sentry_msgs::RefereeInformation referee_info_;
    sentry_msgs::RecordInformation record_info_;
    geometry_msgs::Quaternion odom_quat_;
		geometry_msgs::PointStamped uwb_;
    double odom_time_;
    bool new_odom_ = false;
		bool in_supply_ = false;

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
    transporter::MapDataSendPackage map_data_package_;

    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_; // usb通信接口
    std::thread receive_thread_;

};


} // namespace nav_transporter

#endif