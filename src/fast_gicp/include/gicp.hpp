#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
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


  private:
    void loop(const ros::TimerEvent& event);
    void icp(const ros::TimerEvent& event);

    ros::Subscriber sub_scan_, sub_color_info_;
    ros::Publisher pub_map_, pub_scan_;

    tf::Transform trans_;
		tf::TransformBroadcaster br_;

    PointCloudT::Ptr cloud_target_;
    PointCloudT::Ptr cloud_scan_;
    fast_gicp::FastGICP<PointT, PointT> fgicp_mt_;

    std::mutex mtx_scan_;
    std::mutex mtx_tf_;
		ros::Timer loop_timer_;
		ros::Timer icp_timer_;

    std::string blue_map_pcd_path_;
    std::string red_map_pcd_path_;

    bool save_result_ = false;
    bool pub_result_ = false;
    bool color_init_ = false;
    bool map_init_ = false;
    bool color_info_;
};