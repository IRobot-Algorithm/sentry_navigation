#ifndef STATE_PROCESSING_HPP
#define STATE_PROCESSING_HPP

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

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

		void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

		void globalPathHandler(const nav_msgs::Path::ConstPtr& path);

		bool navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res);
		
		bool navTargetHandler(sentry_srvs::NavTarget::Request &req, sentry_srvs::NavTarget::Response &res);

		void loop(const ros::TimerEvent& event);

    void changeNavExecState(NAV_EXEC_STATE new_state, std::string pos_call);

    void printNavExecState();

		void cutWaypointFromPath();

		ros::Subscriber sub_odom_;
		ros::Subscriber sub_global_path_;
		ros::ServiceServer nav_goal_server;
		ros::ServiceServer nav_target_server;

		ros::Publisher pub_waypoint_;
		ros::Publisher pub_goal_;

		nav_msgs::Path global_path_;
		geometry_msgs::PoseStamped goal_;
		geometry_msgs::PointStamped way_point_;
		nav_msgs::Odometry odom_;

		ros::Timer loop_timer_;

		bool have_odom_ = false;
		bool path_init_ = false;

};


} // namespace nav_transporter

#endif