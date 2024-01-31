#ifndef STATE_PROCESSING_HPP
#define STATE_PROCESSING_HPP

#include <ros/ros.h>
#include <thread>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include "can.hpp"
#include "sentry_srvs/NavGoal.h"
#include "sentry_srvs/NavStop.h"

namespace nav_transporter {

class StateParam {
  public:
    geometry_msgs::PoseStamped goal;
    std::vector<cv::Point2f> area;
    std::vector<cv::Point2f> points;
};

class StateProcess {
	public:
		StateProcess();
		~StateProcess() {};

  	void SubAndPubToROS(ros::NodeHandle &nh);

	private:
		void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void pathHandler(const nav_msgs::Path::ConstPtr& path);

		void velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel);

		bool navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res);

		bool navStopHandler(sentry_srvs::NavStop::Request &req, sentry_srvs::NavStop::Response &res);

		void inline publishStop(const bool& flag);
		
		void sendVelCallback(const ros::TimerEvent& event);
		
		void loop(const ros::TimerEvent& event);

    void receiveCallback();

    void transfer2Quaternion(u_char *buf, double *quaterion);

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_global_path_;
		ros::Subscriber sub_vel;
		ros::ServiceServer nav_goal_server;
		ros::ServiceServer nav_stop_server;

		ros::Publisher pub_waypoint_;
		ros::Publisher pub_goal_;
		ros::Publisher pub_stop_;
		ros::Publisher pub_quat_;

		bool is_stop_;

		geometry_msgs::PoseStamped goal_;
		geometry_msgs::PointStamped way_point_;
		nav_msgs::Odometry odom_;

		std::string config_path_;

		transporter::Can can_;
		
		u_char vel_buf_[7];

		ros::Timer loop_timer_;
		ros::Timer send_vel_timer_;
		ros::Timer tf_timer_;

		std::vector<geometry_msgs::Point> spin_points_;
		ros::Time last_way_time_;
		int iter_;

    std::thread receive_thread_;

};


} // namespace nav_transporter

#endif