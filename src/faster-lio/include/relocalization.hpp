#ifndef RELOCALIZATION_HPP
#define RELOCALIZATION_HPP

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

// pcl
#include <pcl/io/pcd_io.h>  //PCD文件输入输出操作
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Relocalization
{
public:

  Relocalization();
  ~Relocalization();

  void InitParams(ros::NodeHandle &nh);

  bool InitExtrinsic(Eigen::Isometry3d &match_result , PointCloudT::Ptr &scan);

private:

  bool ScanMatchWithICP(Eigen::Isometry3d &trans , PointCloudT::Ptr &cloud_scan, PointCloudT::Ptr &cloud_map);

  void PointCloudObstacleRemoval(PointCloudT::Ptr &cloud_map, PointCloudT::Ptr &cloud_scan, double Distance_Threshold);

  void PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_scan);

  void PointCloudVoxelGridRemoval(PointCloudT::Ptr &cloud_scan, double leafSize);


  pcl::IterativeClosestPoint<PointT, PointT> icp_;

  PointCloudT::Ptr cloud_map_;
  PointCloudT::Ptr cloud_scan_;

  std::string pcd_path_;

  //icp
  double AGE_THRESHOLD_;                  //scan与匹配的最大时间间隔
  double ANGLE_UPPER_THRESHOLD_;          //最大变换角度
  double ANGLE_THRESHOLD_;                //最小变换角度
  double DIST_THRESHOLD_;                 //最小变换距离
  double SCORE_THRESHOLD_MAX_;            //达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛
  double Point_Quantity_THRESHOLD_;       //点云数阈值
  double Maximum_Iterations_;             //ICP中的最大迭代次数
  double VoxelGridRemoval_LeafSize_;      //体素滤波的边长
  double ObstacleRemoval_Distance_Max_;   //最大距离

  bool if_debug_ = true;
  int location_loss_num_ = 0;

};


#endif