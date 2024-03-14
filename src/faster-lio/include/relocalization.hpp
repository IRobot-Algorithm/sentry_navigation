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

class Relocalization
{
public:

  InitParams(ros::Node &nh);

  bool ReLocationWithICP(Eigen::Isometry3d &trans ,const sensor_msgs::PointCloudT::ConstPtr &scan_msg, 
                         PointCloudT::Ptr &cloud_map_msg, const Eigen::Isometry3d &robot_pose);

private:

    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());

    //icp
    double ANGLE_SPEED_THRESHOLD_;          //角速度阈值，大于此值不发布结果
    double AGE_THRESHOLD_;                  //scan与匹配的最大时间间隔
    double ANGLE_UPPER_THRESHOLD_;          //最大变换角度
    double ANGLE_THRESHOLD_;                //最小变换角度
    double DIST_THRESHOLD_;                 //最小变换距离
    double SCORE_THRESHOLD_MAX_;            //达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛
    double Point_Quantity_THRESHOLD_;       //点云数阈值
    double Maximum_Iterations_;             //ICP中的最大迭代次数

};
