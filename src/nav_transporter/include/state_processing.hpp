#ifndef STATE_PROCESSING_HPP
#define STATE_PROCESSING_HPP

#include <ros/ros.h>
#include <thread>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "can.hpp"
#include "sentry_srvs/NavState.h"

namespace nav_transporter {

enum class State : int32_t {
	GOAL = -1,
	PATROL_AREA = 0, 
	SUPPLY_AREA = 1, 
	TRAPEZOIDAL_HEIGHTS = 2, 
	SMALL_RESOURCE_ISLAND = 3, 
	OUR_OUTPOST = 4, 
	ENEMY_OUTPOST = 5, 
};

class StateParam {
  public:
    geometry_msgs::PoseStamped goal;
    std::vector<cv::Point2f> area;
    std::vector<cv::Point2f> points;
};

class StateProcess {
	public:
		StateProcess() {
      memset(vel_buf_, 0, sizeof(vel_buf_));
			state_ = State::PATROL_AREA;
			is_stop_ = false;
			should_stay_ = true;
			iter_ = 0;
			goal_.pose.orientation.w = 1.0;
		}
		~StateProcess() {
			params_.clear();
		};

  	void SubAndPubToROS(ros::NodeHandle &nh);

		bool LoadParams(ros::NodeHandle &nh);

	private:
		void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void pathHandler(const nav_msgs::Path::ConstPtr& path);

		void velHandler(const geometry_msgs::TwistStamped::ConstPtr& vel);

		bool navStateHandler(sentry_srvs::NavState::Request &req, sentry_srvs::NavState::Response &res);

		void inline publishStop(const bool& flag);
		
		void autoNav(const StateParam& param, int& iter);

		void sendVelCallback(const ros::TimerEvent& event);
		
		void loop(const ros::TimerEvent& event);

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_global_path_;
		ros::Subscriber sub_vel;
		ros::ServiceServer nav_state_server;

		ros::Publisher pub_waypoint_;
		ros::Publisher pub_goal_;
		ros::Publisher pub_stop_;

		State state_;
		bool is_stop_;
		bool should_stay_;
		bool in_area_;

		geometry_msgs::PoseStamped goal_;
		geometry_msgs::PointStamped way_point_;
		nav_msgs::Odometry odom_;
    cv::Point2f pose_;

		std::string config_path_;
		std::vector<StateParam> params_;

		transporter::Can can_;
		
		u_char vel_buf_[7];

		ros::Timer loop_timer_;
		ros::Timer send_vel_timer_;
		ros::Time last_way_time_;
		int iter_;

};


} // namespace nav_transporter

#endif