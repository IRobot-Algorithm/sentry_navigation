#ifndef POINTCLOUD_REPUB_HPP
#define POINTCLOUD_REPUB_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace pointcloud_repub{

#define PI = 3.1415926;

class PointCloudProcess {

public:
  PointCloudProcess();

  ~PointCloudProcess() {
    D435_cloud_out_ = nullptr;
  };

  void SubAndPubToROS(ros::NodeHandle &nh);

  bool loadParams(ros::NodeHandle &nh);

  void LivoxMsgHandler(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in);

  void LivoxCloudHandler(const sensor_msgs::PointCloud2ConstPtr& livox_cloud_in);

  void D435CloudHandler(const sensor_msgs::PointCloud2ConstPtr& D435CloudIn);

private:

  void CustomMsg2PointCloud(const livox_ros_driver2::CustomMsg& in, pcl::PointCloud<pcl::PointXYZI>& out);

  bool cutCustomMsg(const livox_ros_driver2::CustomMsg &in, livox_ros_driver2::CustomMsg &out);

  bool transformPointCloud(const std::string &source_frame, const std::string &target_frame, 
                           const pcl::PointCloud<pcl::PointXYZI> &in, pcl::PointCloud<pcl::PointXYZI> &out, 
                           const ros::Time &time, const tf::TransformListener &tf_listener);

private:
  tf::TransformListener tf_;

  ros::Subscriber sub_livox_msg_;
  ros::Subscriber sub_livox_cloud_;
  ros::Subscriber sub_D435_cloud_;

  ros::Publisher pub_livox_msg_;
  ros::Publisher pub_livox_cloud_;
  ros::Publisher pub_D435_cloud_;
  ros::Publisher pub_test_cloud_;
  ros::Publisher pub_registered_cloud_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr D435_cloud_out_;
  
  Eigen::Vector3d extrinT_IMU_BOT_;
  Eigen::Matrix3d extrinR_IMU_BOT_;
  Eigen::Matrix3d extrinR_BOT_IMU_;

  double undecay_dis_;
  double undecay_angle_;
  double undecay_radio_;
  bool adjust_height_;

  std::vector<std::vector<cv::Point>> polygons_;

};


} //namespace pointcloud_repub



#endif