#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <execution>

#include "pointcloud_repub.hpp"

namespace pointcloud_repub{


PointCloudProcess::PointCloudProcess()
{
  D435_cloud_out_  = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

void PointCloudProcess::SubAndPubToROS(ros::NodeHandle &nh)
{
  // ROS subscribe initialization
  sub_livox_msg_ = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 5,
                                            [this](const livox_ros_driver2::CustomMsg::ConstPtr &msg) {LivoxMsgHandler(msg);});

  sub_livox_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 5, 
                                            [this](const sensor_msgs::PointCloud2::ConstPtr &msg) {LivoxCloudHandler(msg);});
 
  sub_D435_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 5,
                                            [this](const sensor_msgs::PointCloud2::ConstPtr &msg) {D435CloudHandler(msg);});

  // ROS publisher initialization
  pub_livox_msg_ = nh.advertise<livox_ros_driver2::CustomMsg>("/livox/repub", 5);
  pub_registered_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);

  // test
  pub_livox_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl", 5);
  pub_D435_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/D435_pcl", 5);
}

bool PointCloudProcess::loadParams(ros::NodeHandle &nh)
{
  std::vector<double> extrinT_IMU_BOT{3, 0.0};  // lidar-imu translation
  std::vector<double> extrinR_IMU_BOT{9, 0.0};  // lidar-imu rotation

  nh.param<std::vector<double>>("lidar/extrinsic_T", extrinT_IMU_BOT, std::vector<double>());
  nh.param<std::vector<double>>("lidar/extrinsic_R", extrinR_IMU_BOT, std::vector<double>());

  extrinT_IMU_BOT_ << extrinT_IMU_BOT[0], extrinT_IMU_BOT[1], extrinT_IMU_BOT[2];
  extrinR_IMU_BOT_ << extrinR_IMU_BOT[0], extrinR_IMU_BOT[1], extrinR_IMU_BOT[2],
                        extrinR_IMU_BOT[3], extrinR_IMU_BOT[4], extrinR_IMU_BOT[5],
                        extrinR_IMU_BOT[6], extrinR_IMU_BOT[7], extrinR_IMU_BOT[8];
  extrinR_BOT_IMU_ = extrinR_IMU_BOT_.inverse();

  return true;

}

bool PointCloudProcess::cutCustomMsg(const livox_ros_driver2::CustomMsg &in, livox_ros_driver2::CustomMsg &out)
{
  for (unsigned int i = 0; i < in.point_num; ++i)
  {
    Eigen::Vector3d pt(in.points[i].x, in.points[i].y, in.points[i].z);
    Eigen::Vector3d res = extrinR_BOT_IMU_ * pt - extrinT_IMU_BOT_;

    // 裁切
    double d = res[0] * res[0] + res[1] * res[1];
    if (!(d < 0.32 * 0.32))
    {
      out.points.push_back(std::move(in.points[i]));
    }
    // if (!(fabs(res[0]) < 0.31 && fabs(res[1]) < 0.31 && res[2] < 1.0))
    // {
    //   out.points.push_back(std::move(in.points[i]));
    // }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  out.header.frame_id = in.header.frame_id;
  out.header.stamp = in.header.stamp;
  out.point_num = out.points.size();

  return true;
}

void PointCloudProcess::CustomMsg2PointCloud(const livox_ros_driver2::CustomMsg& in, pcl::PointCloud<pcl::PointXYZI>& out)
{
  
  // auto time_end = in.points.back().offset_time;
  for (unsigned int i = 0; i < in.point_num; ++i)
  {
    pcl::PointXYZI pt;
    pt.x = in.points[i].x;
    pt.y = in.points[i].y;
    pt.z = in.points[i].z;
    // float s = in.points[i].offset_time / (float)time_end;

    pt.intensity = in.points[i].line + in.points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
    // pt.curvature = s*0.1;
    out.push_back(pt);
  }

  return;
}

bool PointCloudProcess::transformPointCloud(const std::string &source_frame, const std::string &target_frame, 
                                            const pcl::PointCloud<pcl::PointXYZI> &in, pcl::PointCloud<pcl::PointXYZI> &out, 
                                            const ros::Time &time, const tf::TransformListener &tf_listener)
{
  if (source_frame == target_frame)
  {
    out = in;
    return true;
  }

  // if (tf_listener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.1)))
  // {
    tf::StampedTransform transform;
    tf_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    // Convert the TF transform to Eigen format  
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform, eigen_transform);
    pcl::transformPointCloud(in, out, eigen_transform);
    return true;
  // }
    
  // return false;
}

void PointCloudProcess::LivoxMsgHandler(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in)
{
  livox_ros_driver2::CustomMsg livox_msg_cutted;
  // 转换和裁切
  if (cutCustomMsg(*livox_msg_in, livox_msg_cutted));
    pub_livox_msg_.publish(livox_msg_cutted);

  /*
  pcl::PointCloud<pcl::PointXYZI>::Ptr livox_cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
  CustomMsg2PointCloud(livox_msg_cutted, *livox_cloud_out);
  sensor_msgs::PointCloud2 livox_cloud;
  pcl::toROSMsg(*livox_cloud_out, livox_cloud);
  livox_cloud.header.stamp = livox_msg_in->header.stamp;
  livox_cloud.header.frame_id = "lidar_link";
  pub_livox_cloud_.publish(livox_cloud);
  */
}

void PointCloudProcess::LivoxCloudHandler(const sensor_msgs::PointCloud2ConstPtr& livox_cloud_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr livox_cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*livox_cloud_in, *livox_cloud_out);


      pcl::PointCloud<pcl::PointXYZI> registered_cloud_out = *livox_cloud_out + *D435_cloud_out_; //点云融合

      sensor_msgs::PointCloud2 registered_cloud;
      pcl::toROSMsg(registered_cloud_out, registered_cloud);
      registered_cloud.header.stamp = livox_cloud_in->header.stamp;
      registered_cloud.header.frame_id = "map";
      pub_registered_cloud_.publish(registered_cloud);
  
}

void PointCloudProcess::D435CloudHandler(const sensor_msgs::PointCloud2ConstPtr& D435_cloud_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr D435_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*D435_cloud_in, *D435_cloud);

  //对D435的点云进行均匀下采样处理
  pcl::PointCloud<pcl::PointXYZI>::Ptr D435_used_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::UniformSampling<pcl::PointXYZI> us;
  us.setInputCloud(D435_cloud);
  us.setRadiusSearch(0.005f);
  us.filter(*D435_used_cloud);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr D435_radiused_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
  // outrem.setInputCloud(D435_cloud);
  // outrem.setRadiusSearch(0.2);
  // outrem.setMinNeighborsInRadius(1);
  // outrem.filter(*D435_radiused_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr D435_filterd_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> D435;
  D435.setInputCloud(D435_used_cloud);
  D435.setLeafSize(0.05f,0.05f,0.05f);//0.04f,0.04f,0.04f
  D435.filter(*D435_filterd_cloud);

  //对D435点云进行剪裁
  int D435_cloud_size = D435_filterd_cloud->points.size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr D435_cutted_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI point;
  for (int i = 0; i < D435_cloud_size; i++)
  {
    point = D435_filterd_cloud->points[i];
    float dis = sqrt(point.x * point.x + point.y * point.y);
    if (dis<1.0 && dis>0.1)
    {
      D435_cutted_cloud->push_back(std::move(point));
    }
  }

  transformPointCloud(D435_cloud_in->header.frame_id, "map", *D435_cutted_cloud, *D435_cloud_out_, D435_cloud_in->header.stamp, tf_);

  // 发布PointCloud2
  
  // sensor_msgs::PointCloud2 D435_cloud_updated;
  // pcl::toROSMsg(*D435_cutted_cloud, D435_cloud_updated);
  // D435_cloud_updated.header.stamp = D435_cloud_in->header.stamp;
  // D435_cloud_updated.header.frame_id = D435_cloud_in->header.frame_id;
  // pub_D435_cloud_.publish(D435_cloud_updated);
  

}

} // namespace pointcloud_repub

