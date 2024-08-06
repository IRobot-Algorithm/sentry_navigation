#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_config.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <thread>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class GicpLooper
{
  public:
    GicpLooper();

  	void Load(ros::NodeHandle &nh);

    void ScanHandler(const sensor_msgs::PointCloud2ConstPtr& scan);

    void ColorInfoHandler(const std_msgs::Bool::ConstPtr &color);

    void UwbHandler(const geometry_msgs::PointStamped::ConstPtr &uwb);

    void OdomHandler(const nav_msgs::Odometry::ConstPtr &odom);

  private:
    void Loop(const ros::TimerEvent& event);
    
    void Icp(const ros::TimerEvent& event);

    bool Relocalize();

    ros::Subscriber sub_scan_, sub_color_info_, sub_uwb_, sub_odom_;
    ros::Publisher pub_map_, pub_scan_, pub_reboot_;

    tf::Transform trans_;
		tf::TransformBroadcaster br_;

    geometry_msgs::PointStamped uwb_;

    PointCloudT::Ptr cloud_target_;
    PointCloudT::Ptr cloud_scan_;
    fast_gicp::FastGICP<PointT, PointT> fgicp_mt_;
    Eigen::Matrix4f last_result_;
    int icp_failed_time_;

    std::mutex mtx_scan_;
    std::mutex mtx_tf_;
		ros::Timer loop_timer_;
		ros::Timer icp_timer_;

    std::string blue_map_pcd_path_;
    std::string red_map_pcd_path_;

    uint32_t lost_time_ = 0;
    bool save_result_ = false;
    bool pub_result_ = false;
    bool color_init_ = false;
    bool map_init_ = false;
    bool is_init_ = false;
    bool color_info_;
};