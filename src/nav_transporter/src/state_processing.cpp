#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "state_processing.hpp"

namespace nav_transporter {


StateProcess::StateProcess()
{
  exec_state_ = INIT;
			
	goal_.header.frame_id = "map";
	goal_.pose.position.x = 0.0;
	goal_.pose.position.y = 0.0;
	goal_.pose.position.z = 0.0;
	goal_.pose.orientation.x = 0.0;
	goal_.pose.orientation.y = 0.0;
	goal_.pose.orientation.z = 0.0;
	goal_.pose.orientation.w = 1.0;
	point_goal_.header.frame_id = "map";
	point_goal_.point.x = 0.0;
	point_goal_.point.y = 0.0;
	point_goal_.point.z = 0.0;

  way_point_.header.frame_id = "map";
  way_point_.point.x = 0;
  way_point_.point.y = 0;
  way_point_.point.z = 0;
  far_way_point_.header.frame_id = "map";
  far_way_point_.point.x = 0;
  far_way_point_.point.y = 0;
  far_way_point_.point.z = 0;
}

void StateProcess::SubAndPubToROS(ros::NodeHandle &nh)
{
  nh.param<bool>("/state_processing/use_pose_goal", use_pose_goal_, false);
  nh.param<bool>("/state_processing/track_target", track_target_, false);
  nh.param<bool>("/state_processing/use_map", use_map_, false);
  nh.param<bool>("/state_processing/is_test", is_test_, false);
  nh.param<bool>("/state_processing/restricted_track", restricted_track_, true);
  nh.param<std::string>("/state_processing/map_path", map_path_, "");

  if (is_test_)
    exec_state_ = NAVIGATE;

  if (!use_pose_goal_ && !use_map_)
  {
    ROS_INFO("SLAMMING!!!");
    ROS_INFO("SLAMMING!!!");
    ROS_INFO("SLAMMING!!!");
  }
  if (use_map_ && use_pose_goal_)
  {
    ROS_ERROR("Use map but not use far planner!!!");
    ROS_ERROR("Use map but not use far planner!!!");
    ROS_ERROR("Use map but not use far planner!!!");
    use_map_ = false;
  }
  if (use_map_)
    init_map_ = true;

  // ROS subscribe initialization
  this->sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &StateProcess::odometryHandler, this);
  this->sub_global_path_ = nh.subscribe<nav_msgs::Path>("/global_planner/planner/plan", 5, &StateProcess::globalPathHandler, this);
  this->sub_far_waypoint_ = nh.subscribe<geometry_msgs::PointStamped>("/far_way_point", 5, &StateProcess::farWaypointHandler, this);
  this->sub_map_result_ = nh.subscribe<std_msgs::Bool>("/map_result", 5, &StateProcess::mapResultHandler, this);
  this->nav_goal_server = nh.advertiseService("/nav_goal", &StateProcess::navGoalHandler, this);
  this->nav_target_server = nh.advertiseService("/nav_target", &StateProcess::navTargetHandler, this);

  // ROS publisher initialization
  this->pub_waypoint_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  if (use_pose_goal_)
  {
    this->sub_A_star_goal_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, &StateProcess::AstarGoalHandler, this);
    this->pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 5);
  }
  else
  {
    this->pub_map_ = nh.advertise<std_msgs::String>("/read_file_dir", 1);
    this->pub_goal_ = nh.advertise<geometry_msgs::PointStamped>("/goal_point", 5);
    this->pub_map_reset_ = nh.advertise<std_msgs::Bool>("/reset_far_map", 1);
  }

  if (track_target_)
  {
    this->pub_track_dis_ = nh.advertise<std_msgs::Float32>("/track_distance", 1);
    track_dis_.data = 3.0;
  }
  pub_untrack_marker_ = nh.advertise<visualization_msgs::Marker>("/untrack_area", 10);
  
  // /*
  polygons_ = 
  {
    // 三楼
    {
      cv::Point2f(-1.346134, 3.600440),
      cv::Point2f(-1.092801, 1.379447),
      cv::Point2f(-0.781189, 2.5),
      cv::Point2f(7.689121, 2.5),
      cv::Point2f(7.689121, 3.650847)
    },
  };
  // */

  /*
  polygons_ = 
  {
    // 我方台阶
    {
      cv::Point2f(8.236, 6.339),
      cv::Point2f(8.236, 7.480),
      cv::Point2f(6.566, 7.480),
      cv::Point2f(5.521, 5.975),
      cv::Point2f(6.587, 5.230)
    },
    // 敌方台阶
    {
      cv::Point2f(8.408, -7.480),
      cv::Point2f(9.546, -6.038),
      cv::Point2f(8.448, -5.270),
      cv::Point2f(6.799, -6.379),
      cv::Point2f(6.799, -7.520),
    },
    // 我方右侧梯高
    {
      cv::Point2f(-1.959, -4.019),
      cv::Point2f(-1.959, -3.150),
      cv::Point2f(-3.159, -3.150),
      cv::Point2f(-3.159, -4.019)
    },
    // 敌方右侧梯高
    {
      cv::Point2f(18.093, 3.110),
      cv::Point2f(18.093, 3.978),
      cv::Point2f(16.994, 3.978),
      cv::Point2f(16.994, 3.110)
    },
    // 我方左侧梯高
    {
      cv::Point2f(0.739, 6.340),
      cv::Point2f(0.739, 7.480),
      cv::Point2f(-0.261, 7.480),
      cv::Point2f(-0.261, 5.200),
    },
    // 敌方左侧梯高
    {
      cv::Point2f(15.296, -7.480),
      cv::Point2f(15.296, -5.200),
      cv::Point2f(14.296, -6.340),
      cv::Point2f(14.296, -7.480),
    },
  };
  */

  this->loop_timer_ = nh.createTimer(ros::Duration(0.01), &StateProcess::loop, this);
}

void StateProcess::publishPolygons()
{
  int id = 0;
  for (const auto& polygon : polygons_)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "glass";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // Line width
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& point : polygon) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    // Close the loop
    geometry_msgs::Point p;
    p.x = polygon[0].x;
    p.y = polygon[0].y;
    p.z = 0.0;
    marker.points.push_back(p);

    pub_untrack_marker_.publish(marker);
  }
}

void StateProcess::odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = *odom;
  have_odom_ = true;
}

void StateProcess::globalPathHandler(const nav_msgs::Path::ConstPtr& path)
{
  global_path_ = *path;
  path_init_ = true;
}

void StateProcess::farWaypointHandler(const geometry_msgs::PointStamped::ConstPtr& point)
{
  far_way_point_ = *point;
  if (far_way_point_.point.z < -9.0) // 规划失败
    reset_map_ = true; // 清空地图
  else
    reset_map_ = false;
  far_way_point_.point.z = 0;
}

void StateProcess::mapResultHandler(const std_msgs::Bool::ConstPtr& res)
{
  init_map_ = false;
}

void StateProcess::AstarGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  goal_.header.stamp = ros::Time::now();
  goal_.header.frame_id = "map";
  goal_ = *goal;
}

bool StateProcess::navGoalHandler(sentry_srvs::NavGoal::Request &req, sentry_srvs::NavGoal::Response &res)
{
  if (req.pose.pose.position.x < -20.0) // judge save
  {
    geometry_msgs::PointStamped p;
    p.point.x = odom_.pose.pose.position.x;
    p.point.y = odom_.pose.pose.position.y;
    if (p.point.x < 7.5)
      res.is_arrive = true;
    else
      res.is_arrive = false;
    return true;
  }

  goal_.header.stamp = ros::Time::now();
  goal_ = req.pose;

  if (sqrt((odom_.pose.pose.position.x - goal_.pose.position.x) * 
           (odom_.pose.pose.position.x - goal_.pose.position.x) + 
           (odom_.pose.pose.position.y - goal_.pose.position.y) * 
           (odom_.pose.pose.position.y - goal_.pose.position.y)) < 0.2)
    res.is_arrive = true;
  else
    res.is_arrive = false;

  res.odom.pose = odom_.pose.pose;
  
  if (!use_pose_goal_)
  {
    point_goal_.header.stamp = goal_.header.stamp;
    point_goal_.point.x = goal_.pose.position.x;
    point_goal_.point.y = goal_.pose.position.y;
  }
  
  if (!res.is_arrive || exec_state_ == INIT)
    changeNavExecState(NAVIGATE, "desicion");

  return true;
}

bool StateProcess::navTargetHandler(sentry_srvs::NavTarget::Request &req, sentry_srvs::NavTarget::Response &res)
{
  if (track_target_)
  {
    if (!req.is_lost)
    {
      static tf::TransformListener ls;
      tf::StampedTransform map2gimbal_transform;
      ros::Time t = ros::Time().fromSec(ros::Time::now().toSec() - 0.15);
      try {
        ls.lookupTransform("/map", "/world",  
                            t, map2gimbal_transform);
      }
      catch (tf::TransformException &ex) {
        ROS_WARN("TargetTrans : %s",ex.what());
        return true;
      }
      double yaw = tf::getYaw(map2gimbal_transform.getRotation());
      double cos_yaw = cos(yaw);
      double sin_yaw = sin(yaw);
      way_point_.point.x = map2gimbal_transform.getOrigin().x() +
                          req.pose.pose.position.x * cos_yaw -
                          req.pose.pose.position.y * sin_yaw;
      way_point_.point.y = map2gimbal_transform.getOrigin().y() +
                          req.pose.pose.position.x * sin_yaw +
                          req.pose.pose.position.y * cos_yaw;
      target_z_ = map2gimbal_transform.getOrigin().z() + req.pose.pose.position.z + 0.35; // 雷达距地面高度0.35
      
      double distance = sqrt(pow(req.pose.pose.position.x, 2) + pow(req.pose.pose.position.y, 2));
      if (distance < req.distance)
      {
        double diff_x = (map2gimbal_transform.getOrigin().x() - way_point_.point.x) * 2 * req.distance / distance;
        double diff_y = (map2gimbal_transform.getOrigin().y() - way_point_.point.y) * 2 * req.distance / distance;
        way_point_.point.x += diff_x;
        way_point_.point.y += diff_y;
      }
    }

    if (req.restricted_area == 4)
    {
      way_point_.point.x = req.pose.pose.position.x;
      way_point_.point.y = req.pose.pose.position.y;
      target_z_ = req.pose.pose.position.z;
    }

    // rmuc 在对面补给区
    if (way_point_.point.x > 19.0 && way_point_.point.x < 21.3 &&
        way_point_.point.y > 5.0 && way_point_.point.y < 8.0)
    {
      res.success = false;
      return true;
    }

    way_point_.point.z = -0.1;
    if (!req.is_dynamic) // static
    {
      way_point_.point.z -= 0.2;
    }
    else // dynamic
    {
      /*
      if (target_z_ < 0.0 || target_z_ > 0.4 || odom_.pose.pose.position.z > 0.16) // 高度不一致
      {
        res.success = false;
        return true;
      }
      // if (!isPointInsidePolygon(way_point_, polygon_)) // 不在区域内 不跟随
      if (!((odom_.pose.pose.position.x < 4.5) || (odom_.pose.pose.position.x > 12.0) ||
            (odom_.pose.pose.position.y > -2.0 && odom_.pose.pose.position.y < 2.0))) // 不在区域内 不跟随
      {
        res.success = false;
        return true;
      }
      */
      if (restricted_track_)
      {
        for (const auto& polygon : polygons_)
        {
          if(cv::pointPolygonTest(polygon, 
             cv::Point2f(odom_.pose.pose.position.x, odom_.pose.pose.position.y), false) >= 0)
          {
            res.success = false;
            return true;
          }
        }
      }
    }

    track_dis_.data = req.distance;
    pub_track_dis_.publish(track_dis_);

  }
  
  changeNavExecState(TRACK, "desicion");

  res.success = true;
  return true;
}

void StateProcess::loop(const ros::TimerEvent& event)
{

  static int nav_num = 0;
  nav_num++;
  if (nav_num == 100)
  {
    printNavExecState();
    if (!have_odom_)
      ROS_WARN("no odom.");
    else
    {
      if (init_map_ && !use_pose_goal_ && use_map_)
      {
        std_msgs::String msg;
        msg.data = map_path_;
        pub_map_.publish(msg);
      }
      if (reset_map_ && !use_pose_goal_)
      {
        std_msgs::Bool msg;
        msg.data = true;
        pub_map_reset_.publish(msg);
      }
    }
    // publishPolygons();
    nav_num = 0;
  }

  switch (exec_state_)
  {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      break;
    }
    case TRACK:
    {
      if (!track_target_)
      {
        way_point_.point.x = odom_.pose.pose.position.x;
        way_point_.point.y = odom_.pose.pose.position.y;
        way_point_.point.z = 0;
      }
      if (way_point_.point.z < -0.05 && way_point_.point.z > -0.25) // tracking
      {
        if (!((odom_.pose.pose.position.x < 4.5) || (odom_.pose.pose.position.x > 12.0) ||
              (odom_.pose.pose.position.y > -2.0 && odom_.pose.pose.position.y < 2.0)))
        {
          way_point_.point.z -= 0.2; // static
        }
      }
      pub_waypoint_.publish(way_point_);
      break;
    }
    case NAVIGATE:
    {
      if (use_pose_goal_)
      {
        pub_goal_.publish(goal_);
        if (path_init_)
        {
          cutWaypointFromPath();
          pub_waypoint_.publish(way_point_);
        }
        // TODO: else ...
      }
      else
      {
        if (!is_test_)
          pub_goal_.publish(point_goal_);
        pub_waypoint_.publish(far_way_point_);
      }
      break;
    }
  }

}

void StateProcess::changeNavExecState(NAV_EXEC_STATE new_state, std::string pos_call)
{
  if (new_state == exec_state_)
    return;
  static std::string state_str[3] = {"INIT", "TRACK", "NAVIGATE"};
  int pre_s = int(exec_state_);
  if (new_state == NAVIGATE && exec_state_ != NAVIGATE)
    path_init_ = false;
  exec_state_ = new_state;
  ROS_INFO("[ %s ]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void StateProcess::printNavExecState()
{
  static std::string state_str[3] = {"INIT", "TRACK", "NAVIGATE"};
  
  ROS_INFO("[NAV]: state: %s", state_str[int(exec_state_)].c_str());
}

void StateProcess::cutWaypointFromPath()
{
  unsigned int PathLength;
	PathLength = global_path_.poses.size();
	if (PathLength != 0)
	{
    std::vector<geometry_msgs::PoseStamped>::const_iterator it = global_path_.poses.begin();

    if (PathLength > 31)//81
    {
      it = global_path_.poses.begin() + 29;//79
    }
    else
    {
      it = global_path_.poses.end() - 1;
    }

    way_point_.point.x = it->pose.position.x;
    way_point_.point.y = it->pose.position.y;
    way_point_.point.z = 0;
    way_point_.header.stamp = ros::Time().now();
    way_point_.header.frame_id = "map";
	}
}


} // namespace nav_transporter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_processing");
  ros::NodeHandle nh;

  nav_transporter::StateProcess StateProcessObj;
  StateProcessObj.SubAndPubToROS(nh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}