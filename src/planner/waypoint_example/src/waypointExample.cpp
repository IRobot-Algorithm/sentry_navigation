#include <math.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <string.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
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

using namespace std;

const double PI = 3.1415926;

string waypoint_file_dir;
string boundary_file_dir;
double waypointXYRadius = 0.5;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;
int path_state = 0;
int wayPointID = 0;

// reading waypoints from file function
void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(waypoint_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(waypoint_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  waypoints->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(waypoint_file, "%f", &point.x);
    val2 = fscanf(waypoint_file, "%f", &point.y);
    val3 = fscanf(waypoint_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }
    cout<<"123"<<endl;
    waypoints->push_back(point);
  }

  fclose(waypoint_file);
}

// reading boundary from file function
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    boundary->push_back(point);
  }

  fclose(boundary_file);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}

void pathHandler(const std_msgs::Int16::ConstPtr& indexMsgs)
{
  path_state = indexMsgs->data;
  ROS_INFO("path_state: %d\n", path_state);
  // string s = to_string(path_state);
  if(path_state==1)
  waypoint_file_dir = "/home/nuc/Desktop/exploration3_ws/src/planner/waypoint_example/data/waypoints_garage1.ply";
  if(path_state==2)
  waypoint_file_dir = "/home/nuc/Desktop/exploration3_ws/src/planner/waypoint_example/data/waypoints_garage2.ply";
  if(path_state==3)
  waypoint_file_dir = "/home/nuc/Desktop/exploration3_ws/src/planner/waypoint_example/data/waypoints_garage3.ply";
  if(path_state==4)
  waypoint_file_dir = "/home/nuc/Desktop/exploration3_ws/src/planner/waypoint_example/data/waypoints_garage4.ply";
  if(path_state==5)
  waypoint_file_dir = "/home/nuc/Desktop/exploration3_ws/src/planner/waypoint_example/data/waypoints_garage5.ply";
  if(path_state!=0)
  {
  readWaypointFile();
  wayPointID = 0;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointExample");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);
  nhPrivate.getParam("waypointZBound", waypointZBound);
  nhPrivate.getParam("waitTime", waitTime);
  nhPrivate.getParam("frameRate", frameRate);
  nhPrivate.getParam("speed", speed);
  nhPrivate.getParam("sendSpeed", sendSpeed);
  nhPrivate.getParam("sendBoundary", sendBoundary);

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  //决策状态 zbh
  ros::Subscriber subPath_index = nh.subscribe<std_msgs::Int16> ("/path_index", 1 , pathHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  geometry_msgs::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";

  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> ("/speed", 5);
  std_msgs::Float32 speedMsgs;

  ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5);
  geometry_msgs::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  // read waypoints from file
  // readWaypointFile();


  // read boundary from file
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  // int wayPointID = 0;
  // int waypointSize = waypoints->points.size();

  // if (waypointSize == 0) {
  //   printf ("\nNo waypoint available, exit.\n\n");
  //   exit(1);
  // }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    // int path_index = 0;
    //   waypoint_file_dir = "/home/nuc/Desktop/exploration5/src/planner/waypoint_example/data/waypoints_garage.ply";
    // if(waypoint_file_dir != "")
    // {
    // readWaypointFile();
    // }
    int waypointSize = waypoints->points.size();
    if(waypointSize>0)
    {
    float disX = vehicleX - waypoints->points[wayPointID].x;
    float disY = vehicleY - waypoints->points[wayPointID].y;
    float disZ = vehicleZ - waypoints->points[wayPointID].z;

    // start waiting if the current waypoint is reached
    // if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
    //   waitTimeStart = curTime;
    //   isWaiting = true;
    // }

    // // move to the next waypoint after waiting is over
    // if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
    //   wayPointID++;
    //   isWaiting = false;
    // }
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && wayPointID < waypointSize - 1) {
      wayPointID++;
    }


    // publish waypoint, speed, and boundary messages at certain frame rate
    // if (curTime - waypointTime > 1.0 / frameRate) {
      // if (!isWaiting) {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs.point.x = waypoints->points[wayPointID].x;
        waypointMsgs.point.y = waypoints->points[wayPointID].y;
        waypointMsgs.point.z = waypoints->points[wayPointID].z;
        pubWaypoint.publish(waypointMsgs);
      // }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed.publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);
        pubBoundary.publish(boundaryMsgs);
      }

      // waypointTime = curTime;
    // }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
