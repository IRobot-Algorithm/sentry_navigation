#include <ros/ros.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

#include "gicp.hpp"

tf::Transform eigenMatrixToTf(const Eigen::Matrix4f& matrix)
{
  tf::Matrix3x3 rotation;
  tf::Vector3 translation;

  for (int i = 0; i < 3; ++i)
  {
      for (int j = 0; j < 3; ++j)
      {
          rotation[i][j] = matrix(i, j);
      }
  }

  translation.setX(matrix(0, 3));
  translation.setY(matrix(1, 3));
  translation.setZ(matrix(2, 3));

  tf::Transform transform(rotation, translation);
  return transform;
}

GicpLooper::GicpLooper()
{
  cloud_target_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
  cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
  fgicp_mt_.setNumThreads(4);
  trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  trans_.setOrigin(tf::Vector3(0, 0, 0));
}

void GicpLooper::Load(ros::NodeHandle &nh)
{

  nh.param<bool>("icp/pub_result", pub_result_, false);
  nh.param<std::string>("icp/blue_map_pcd_path", blue_map_pcd_path_, "");
  nh.param<std::string>("icp/red_map_pcd_path", red_map_pcd_path_, "");

  sub_scan_ = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 5, 
                                            [this](const sensor_msgs::PointCloud2::ConstPtr &msg) {ScanHandler(msg);});

  if (pub_result_)
  {
      pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/icp_cloud_map", 1);
      pub_scan_ = nh.advertise<sensor_msgs::PointCloud2>("/icp_cloud_scan", 1);
  }

  this->loop_timer_ = nh.createTimer(ros::Duration(0.01), &GicpLooper::loop, this);
  this->icp_timer_ = nh.createTimer(ros::Duration(2.0), &GicpLooper::icp, this);

}

void GicpLooper::loop(const ros::TimerEvent& event)
{
  mtx_tf_.lock();
  br_.sendTransform(tf::StampedTransform(trans_, ros::Time::now(), "map", "odom"));
  mtx_tf_.unlock();
}

void GicpLooper::icp(const ros::TimerEvent& event)
{
  
  // downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.5f, 0.5f, 0.5f);

  PointCloudT::Ptr cloud_source(new PointCloudT());
  mtx_scan_.lock();
  voxelgrid.setInputCloud(cloud_scan_);
  voxelgrid.filter(*cloud_source);
  cloud_scan_->clear();
  mtx_scan_.unlock();

  if (!color_init_)
    return;

  if (!map_init_)
  {
    std::string map_pcd_path;
    if (color_info_) // blue
      map_pcd_path = blue_map_pcd_path_;
    else // red
      map_pcd_path = red_map_pcd_path_;

    PointCloudT::Ptr cloud_map(new PointCloudT());
    if (pcl::io::loadPCDFile<PointT>(map_pcd_path, *cloud_map) == -1)
    {
        ROS_ERROR("Couldn't read PCD file");
        return;
    }
    voxelgrid.setInputCloud(cloud_map);
    voxelgrid.filter(*cloud_target_);
    map_init_ = true;
  }

  PointCloudT::Ptr aligned(new PointCloudT);

  double fitness_score = 0.0;

  auto t1 = std::chrono::high_resolution_clock::now();
  // fgicp_mt_.clearSource();
  fgicp_mt_.setInputTarget(cloud_target_);
  fgicp_mt_.setInputSource(cloud_source);
  fgicp_mt_.align(*aligned);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = fgicp_mt_.getFitnessScore();
  double cost = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

  std::cout << "icp_cost:" << cost << "[msec] " << std::flush;
  std::cout << "fitness_score:" << fitness_score << std::flush;

  if (fitness_score < 1.0)
  {
    mtx_tf_.lock();
    trans_ = eigenMatrixToTf(fgicp_mt_.getFinalTransformation()).inverse();
    mtx_tf_.unlock();

    if (pub_result_)
    {

      sensor_msgs::PointCloud2 map_msg;
      pcl::toROSMsg(*cloud_target_, map_msg);
      map_msg.header.frame_id = "map";
      pub_map_.publish(map_msg);

      sensor_msgs::PointCloud2 scan_msg;
      pcl::toROSMsg(*aligned, scan_msg);
      scan_msg.header.frame_id = "map";
      pub_scan_.publish(scan_msg);

    }
  }

}

void GicpLooper::ScanHandler(const sensor_msgs::PointCloud2ConstPtr& scan)
{
  PointCloudT cloud;
  pcl::fromROSMsg(*scan, cloud);

  mtx_scan_.lock();
  *cloud_scan_ += cloud;
  mtx_scan_.unlock();
}

void GicpLooper::ColorInfoHandler(const std_msgs::Bool::ConstPtr& color)
{
  color_info_ = color->data;
  color_init_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gicp_node");
  ros::NodeHandle nh;

  GicpLooper GicpLooperObj;
  GicpLooperObj.Load(nh);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}