#ifndef STATE_PROCESSING_HPP
#define STATE_PROCESSING_HPP

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "sentry_srvs/NavGoal.h"
#include "sentry_srvs/NavTarget.h"

namespace nav_transporter {

enum NAV_EXEC_STATE {
	INIT, 
	TRACK,
	NAVIGATE,
};

class StateProcess {
	public:

		StateProcess();
		~StateProcess() {};

  	void SubAndPubToROS(ros::NodeHandle &nh);

	private:
		NAV_EXEC_STATE exec_state_;

	  void publishPolygons();

		void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void globalPathHandler(const nav_msgs::Path::ConstPtr& path);

		void farWaypointHandler(const geometry_msgs::PointStamped::ConstPtr& point);

		void mapResultHandler(const std_msgs::Bool::ConstPtr& res);

		void AstarGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& goal);

		/*
		* @brief 决策导航信息服务器
		* @auther wyq
		*/
		bool navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res);

		/*
		* @brief 决策跟随信息服务器
		* @auther wyq
		*/
		bool navTargetHandler(sentry_srvs::NavTarget::Request &req, sentry_srvs::NavTarget::Response &res);

		/*
		* @brief 状态机主循环
		* @auther wyq
		*/
		void loop(const ros::TimerEvent& event);

    void changeNavExecState(NAV_EXEC_STATE new_state, std::string pos_call);

    void printNavExecState();

		/*
		* @brief 从全局路径中采局部点
		* @auther wyq
		*/
		void cutWaypointFromPath();

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_global_path_;
		ros::Subscriber sub_far_waypoint_;
		ros::Subscriber sub_map_result_;
		ros::Subscriber sub_A_star_goal_;
		ros::ServiceServer nav_goal_server;
		ros::ServiceServer nav_target_server;

		ros::Publisher pub_goal_;
		ros::Publisher pub_map_;
		ros::Publisher pub_waypoint_;
		ros::Publisher pub_map_reset_;
		ros::Publisher pub_track_dis_;
  	ros::Publisher pub_untrack_marker_;

		bool use_map_ = false;
		bool reset_map_ = false;
		bool init_map_ = false;
		std::string map_path_;
		std_msgs::Float32 track_dis_;
		nav_msgs::Path global_path_;
		geometry_msgs::PoseStamped goal_;
		geometry_msgs::PointStamped point_goal_;
		geometry_msgs::PointStamped way_point_;
		geometry_msgs::PointStamped far_way_point_;
		nav_msgs::Odometry odom_;
  	std::vector<std::vector<cv::Point2f>> polygons_;
		double target_z_ = 0.0;

		ros::Timer loop_timer_;

		bool use_pose_goal_;
		bool track_target_;
		bool have_odom_ = false;
		bool path_init_ = false;
		bool is_test_ = false;
		bool restricted_track_ = true;

};


} // namespace nav_transporter

#endif