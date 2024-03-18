#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
//double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
//bool noRotAtStop = false;
//bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float desiredYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;
float velocityX = 0;
float velocityY = 0;
float velocityYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool odomInit = false;
bool navFwd = true;
double switchTime = 0;

double goalX = 0.0;
double goalY = 0.0;
double goalZ = 0.0;

double v_kp = 10.0;
double v_ki = 0.0;
double v_kd = 0.1;

float I_x = 0.0;
float I_y = 0.0;

float last_err_x = 0;
float last_err_y = 0;

nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  
  tf::Quaternion odomQuat = tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);
  tf::Matrix3x3(odomQuat).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x;
  vehicleY = odomIn->pose.pose.position.y;
  vehicleZ = odomIn->pose.pose.position.z;
  velocityX = odomIn->twist.twist.linear.x;
  velocityY = odomIn->twist.twist.linear.y;

  odomInit = true;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }

}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  //if (joy->axes[4] == 0) joySpeed = 0;
  //joyYaw = joy->axes[3];
  //if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  /*if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }*/

  joyYaw = joy->axes[0];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Bool::ConstPtr& stop)
{
  safetyStop = stop->data;
}

//zbh接收航点信息
void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  //nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  //nhPrivate.getParam("noRotAtStop", noRotAtStop);
  //nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);
  nhPrivate.getParam("goalZ", goalZ);
  nhPrivate.getParam("v_kp", v_kp);
  nhPrivate.getParam("v_ki", v_ki);
  nhPrivate.getParam("v_kd", v_kd);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/Odometry", 1, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/nav_path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Bool> ("/stop", 5, stopHandler);

  //订阅航点（位姿）
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  //发布导航状态
  ros::Publisher pubnavState = nh.advertise<std_msgs::Bool> ("/nav_state", 5);
  std_msgs::Bool navpoint_stateMsgs;

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "world";

  double worldRoll, worldPitch, worldYaw;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  static tf::TransformBroadcaster br;
  static tf::TransformListener ls;
  ros::Rate rate(200);
  bool status = ros::ok();
  while (status) {
    rate.sleep();
    ros::spinOnce();
    if (pathInit && odomInit) {
      tf::StampedTransform transform;
      try{
        ls.lookupTransform("/map_link", "/world",  
                                ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
      }

      tf::Matrix3x3 rotation(transform.getRotation());
      rotation.getRPY(worldRoll, worldPitch, worldYaw);

        // cmd_vel.twist.linear.x = 0.5;
        // cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.linear.z = 1.0;
        // cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
        // pubSpeed.publish(cmd_vel);
        continue;

      int pathSize = path.poses.size();
      float endDisX = goalX - vehicleX;
      float endDisY = goalY - vehicleY;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      
      // if (endDis < 4.0 && goalZ < -1e4) // tracking
      // {
      //   cmd_vel.twist.linear.x = 0.0;
      //   cmd_vel.twist.linear.y = 0.0;
      //   cmd_vel.twist.linear.z = 1.0;
      //   cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
      //   pubSpeed.publish(cmd_vel);
      //   continue;
      // }
      if (endDis < 0.2) // navigating
      {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.linear.z = 1.0;
        cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
        pubSpeed.publish(cmd_vel);
        continue;
      }
      if (endDis > 0.2 && pathSize <= 5)
      {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.linear.z = 1.0;
        cmd_vel.twist.angular.z = vehicleYaw - worldYaw + 0.1;
        pubSpeed.publish(cmd_vel);
        continue;
      }

      float dis_x, dis_y, dis;
      while (pathPointID < pathSize - 1)
      {
        dis_x = path.poses[pathPointID].pose.position.x - vehicleX;
        dis_y = path.poses[pathPointID].pose.position.y - vehicleY;
        dis = sqrt(dis_x * dis_x + dis_y * dis_y);
        if (dis < lookAheadDis)
        {
          pathPointID++;
        }
        else
        {
          break;
        }
      }
      dis_x = path.poses[pathPointID].pose.position.x - vehicleX;
      dis_y = path.poses[pathPointID].pose.position.y - vehicleY;
      dis = sqrt(dis_x * dis_x + dis_y * dis_y);

      // float next_dis_x, next_dis_y, next_dis;
      // int nextPathID = pathPointID + 20;
      // if (nextPathID >= pathSize - 1)
      // {
      //   next_dis_x = path.poses[nextPathID].pose.position.x - vehicleX;
      //   next_dis_y = path.poses[nextPathID].pose.position.y - vehicleY;
      //   next_dis = sqrt(next_dis_x * next_dis_x + next_dis_y * next_dis_y);
      // }

      // next_dis_x = path.poses[nextPathID].pose.position.x - vehicleX;
      // next_dis_y = path.poses[nextPathID].pose.position.y - vehicleY;
      // next_dis = sqrt(next_dis_x * next_dis_x + next_dis_y * next_dis_y);

      // float target_vx = 0.95 * v_kp * dis_x + 0.05 * v_kp * next_dis_x;
      // float target_vy = 0.95 * v_kp * dis_y + 0.05 * v_kp * next_dis_y;
      I_x += dis_x;
      I_y += dis_y;
      float speed_x = v_kp * dis_x + v_ki * I_x + v_kd * (last_err_x - dis_x);
      float speed_y = v_kp * dis_y + v_ki * I_y + v_kd * (last_err_y - dis_y);

      // float speed_x = v_kp * target_vx;
      // float speed_y = v_kp * target_vy;

      float speed = sqrt(speed_x * speed_x + speed_y * speed_y);
      last_err_x = dis_x;
      last_err_y = dis_y;
      
      if (speed > maxSpeed)
      {
        speed_x *= maxSpeed / speed;
        speed_y *= maxSpeed / speed;
      }

      // angle
      float pathDir = atan2(speed_y, speed_x);
      float yawDiff = pathDir - worldYaw;

      if (yawDiff > PI) 
        yawDiff -= 2 * PI;
      else if (yawDiff < -PI) 
        yawDiff += 2 * PI;
      if (yawDiff > PI) 
        yawDiff -= 2 * PI;
      else if (yawDiff < -PI) 
        yawDiff += 2 * PI;
      // if (twoWayDrive) {
      //   double time = ros::Time::now().toSec();
      //   if (fabs(yawDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
      //     navFwd = false;
      //     switchTime = time;
      //   } else if (fabs(yawDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
      //     navFwd = true;
      //     switchTime = time;
      //   }
      // }

      // if (!navFwd) {
      //   yawDiff += PI;
      //   if (yawDiff > PI) yawDiff -= 2 * PI;
      // }

      if (fabs(pathDir - vehicleYaw) > PI / 5)
      {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.linear.z = 1.0;
        cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
        // cmd_vel.twist.angular.z = yawDiff;
        pubSpeed.publish(cmd_vel);
        continue;
      }

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.twist.linear.x = cos(worldYaw) * speed_x + sin(worldYaw) * speed_y;
        cmd_vel.twist.linear.y = -sin(worldYaw) * speed_x + cos(worldYaw) * speed_y;
        cmd_vel.twist.linear.z = 0.0;
        cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
        // cmd_vel.twist.angular.z = yawDiff;
        pubSpeed.publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }
    else
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      // cmd_vel.twist.linear.z = 1.0;
      cmd_vel.twist.angular.z = vehicleYaw - worldYaw;
      pubSpeed.publish(cmd_vel);
    }

    status = ros::ok();
  
  }

  return 0;
}
