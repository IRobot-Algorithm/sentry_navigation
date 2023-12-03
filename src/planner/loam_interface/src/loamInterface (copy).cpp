#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/Odometry";
// string registeredScanTopic = "/velodyne_cloud_registered";
// string stateEstimationTopic = "/livox_odometry_mapped";
string registeredScanTopic = "/livox/lidar";//"/livox/lidar"
bool flipStateEstimation = false;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation) {
    // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    // pitch = -pitch;
    // yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;

    // odomData.pose.pose.position.x = odom->pose.pose.position.z;
    // odomData.pose.pose.position.y = odom->pose.pose.position.x;
    // odomData.pose.pose.position.z = odom->pose.pose.position.y;
    odomData.pose.pose.position.x = odom->pose.pose.position.x;
    odomData.pose.pose.position.y = odom->pose.pose.position.y;
    odomData.pose.pose.position.z = odom->pose.pose.position.z;
  }

  // publish odometry messages
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z+0.20));//0.25

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

//坐标系转换
//坐标变换将激光雷达坐标系下的点转换到map坐标系下
void changePoint(pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud,
                 double yaw,double pitch,double roll,double x,double y,double z)
{
    Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transformYaw;
    Eigen::Matrix4f transformPitch;
    Eigen::Matrix4f transformRoll;
    
    //航向角
    transformYaw<<cos(yaw),-sin(yaw),0,0,\
            sin(yaw),cos(yaw),0,0,\
            0,0,1,0,\
            0,0,0,1;
    //俯仰角
    transformPitch<<cos(pitch),0,sin(pitch),0,\
            0,1,0,0,\
            -sin(pitch),0,cos(pitch),0,\
            0,0,0,1;
    //横滚角
    transformRoll<<1,0,0,0,\
            0,cos(roll),-sin(roll),0,\
            0,sin(roll),cos(roll),0,\
            0,0,0,1;
            
    //旋转矩阵
    transform=transformRoll*transformPitch*transformYaw;
    
    //平移矩阵
    transform(0,3)=x;
    transform(1,3)=y;
    transform(2,3)=z;
    
    //坐标转换
    pcl::transformPointCloud(*lidarCloud,*mapCloud,transform);
    // std::cout<<"frame_id"<<mapCloud->header.frame_id<<endl;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  // if (flipRegisteredScan) {
  //   int laserCloudSize = laserCloud->points.size();
  //   for (int i = 0; i < laserCloudSize; i++) {
  //     float temp = laserCloud->points[i].x;
  //     laserCloud->points[i].x = laserCloud->points[i].z;
  //     laserCloud->points[i].z = laserCloud->points[i].y;
  //     laserCloud->points[i].y = temp;
  //   }
  // }

  //点云坐标系转换：sensor->map
  double roll, pitch, yaw, x, y, z;
  geometry_msgs::Quaternion geoQuat = odomData.pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  pitch = pitch;
  yaw = yaw;
  roll = roll;
  x=odomData.pose.pose.position.x;
  y=odomData.pose.pose.position.y;
  z=odomData.pose.pose.position.z+0.20;//0.25
  changePoint(laserCloud,laserCloud,yaw,pitch,roll,x,y,z);



  // publish registered scan messages
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = "map"; //map  注意frame_id
  pubLaserCloudPointer->publish(laserCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}
