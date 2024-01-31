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

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
bool odomInit = false;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

//zbh
float goalX = 10.0;
float goalY = 10.0;
float vyaw = 0.0;

std::mutex mtx_buffer;
std::deque<geometry_msgs::QuaternionStamped::ConstPtr> quat_buffer;
double last_timestamp_quat;

float worldRoll; // C_board_link
float worldPitch; // C_board_link
float worldYaw; // C_board_link
double quat_time;

tf::Quaternion odomQuat;
tf::Quaternion C_board_quat;
tf::Transform trans;
bool new_odom;

nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  
  odomQuat = tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);
  tf::Matrix3x3(odomQuat).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x;
  vehicleY = odomIn->pose.pose.position.y;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }

  if (!odomInit) {
    desiredYaw = yaw;
    odomInit = true;
  }

  new_odom = true;
  // tf::TransformListener ls;
  // tf::StampedTransform tr;
  // try
  // {
  //     ls.lookupTransform("world", "base_link", ros::Time(0), tr);
  // }
  // catch (tf::LookupException &e)
  // {
  //     ROS_WARN("%s", e.what ());
  //     // cmd_vel.twist.linear.x = 0.0;
  //     // cmd_vel.twist.linear.y = 0.0;
  //     // // cmd_vel.twist.angular.z = vehicleYawRate;
  //     // cmd_vel.twist.angular.z = 0;
  //     // pubSpeed.publish(cmd_vel);
  //     return;      
  // }
  // catch (tf::ExtrapolationException &e)
  // {
  //     ROS_WARN("%s", e.what ());
  //     // cmd_vel.twist.linear.x = 0.0;
  //     // cmd_vel.twist.linear.y = 0.0;
  //     // // cmd_vel.twist.angular.z = vehicleYawRate;
  //     // cmd_vel.twist.angular.z = 0;
  //     // pubSpeed.publish(cmd_vel);
  //     return;
  // }
  // ROS_INFO("in");

  // tf::Matrix3x3(tr.getRotation()).getRPY(roll, pitch, yaw);//进行转换
  // worldRoll = roll;
  // worldPitch = pitch;
  // worldYaw = yaw;
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

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

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

void quatHandler(const geometry_msgs::QuaternionStamped::ConstPtr& quat)
{
  geometry_msgs::QuaternionStamped::Ptr msg(new geometry_msgs::QuaternionStamped(*quat));
  double timestamp = msg->header.stamp.toSec();

  mtx_buffer.lock();
  if (timestamp < last_timestamp_quat)
  {
    ROS_WARN("quat loop back, clear buffer");
    quat_buffer.clear();
  }

  last_timestamp_quat = timestamp;
  quat_buffer.emplace_back(msg);
  mtx_buffer.unlock();

  while (quat_buffer.size() > 1000)
    quat_buffer.pop_front();
}

//zbh接收航点信息
void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

bool syncPackages()
{
  if (!new_odom || quat_buffer.empty()) {
    return false;
  }

  double odom_time = odomTime;
  double vehicle_roll = vehicleRoll;
  double vehicle_pitch = vehiclePitch;
  double vehicle_yaw = vehicleYaw;
  tf::Quaternion odom_quat = odomQuat;

  geometry_msgs::QuaternionStamped::ConstPtr last_quat = quat_buffer.front(); 
  geometry_msgs::QuaternionStamped::ConstPtr next_quat; 
  double quat_time = quat_buffer.front()->header.stamp.toSec();
  while ((!quat_buffer.empty()) && (quat_time < odom_time))
  {
      quat_time = quat_buffer.front()->header.stamp.toSec();
      if (quat_time > odom_time) 
      {
        next_quat = quat_buffer.front();
        break;
      }
      last_quat = quat_buffer.front();
      quat_buffer.pop_front();
  }

  if (quat_buffer.empty() || last_quat == nullptr || next_quat == nullptr)
    return true;

 
  double t1 = last_quat->header.stamp.toSec();
  double t2 = next_quat->header.stamp.toSec();

  if (t2 - t1 > 0.01 || t2 - t1 < 0)
    return true; // may be problem

  float radio = (odom_time - t1) / (t2 - t1);

  tf::Quaternion q1, q2;
  tf::quaternionMsgToTF(last_quat->quaternion, q1);
  tf::quaternionMsgToTF(next_quat->quaternion, q2);

  C_board_quat =  q1.slerp(q2, radio) * odom_quat;
  // C_board_quat =  q1 * odom_quat;
  C_board_quat.normalize();
  trans.setRotation(C_board_quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(C_board_quat).getRPY(roll, pitch, yaw);

  quat_time = odom_time;
  worldRoll = roll;
  worldPitch = pitch;
  worldYaw = yaw;

  ROS_INFO("%f", worldYaw);

  new_odom = false;
  return true;
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

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/Odometry", 1, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/nav_path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Bool> ("/stop", 5, stopHandler);

  ros::Subscriber subQuat = nh.subscribe<geometry_msgs::QuaternionStamped> ("/world_quat", 5, quatHandler);

  //订阅航点（位姿）
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  //发布导航状态
  ros::Publisher pubnavState = nh.advertise<std_msgs::Bool> ("/nav_state", 5);
  std_msgs::Bool navpoint_stateMsgs;

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  // cmd_vel.header.frame_id = "map";//"vehicle"
  cmd_vel.header.frame_id = "world";
  trans.setOrigin(tf::Vector3(0, 0, 0));

  new_odom = false;

  double tf_roll, tf_pitch, tf_yaw;
  geometry_msgs::Quaternion geoQuat_tf;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  static tf::TransformBroadcaster br;

  ros::Rate rate(200);//100
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                          + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = goalX - vehicleX;
      float endDisY = goalY - vehicleY;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      desiredYaw += 0.01 * joyYaw;
      if (desiredYaw > PI) desiredYaw -= 2 * PI;
      else if (desiredYaw < -PI) desiredYaw += 2 * PI;

      float yawDiff = vehicleYaw - desiredYaw;
      if (yawDiff > PI) yawDiff -= 2 * PI;
      else if (yawDiff < -PI) yawDiff += 2 * PI;

      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(yawDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(yawDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        yawDiff += PI;
        if (yawDiff > PI) yawDiff -= 2 * PI;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * yawDiff;
      else vehicleYawRate = -yawRateGain * yawDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      /*if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }*/

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if ((endDis / slowDwnDisThre) < joySpeed) {
        joySpeed2 *= (endDis / slowDwnDisThre);
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if (dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      float mapSpeedX = cos(dirDiff) * vehicleSpeed;
      float mapSpeedY = -sin(dirDiff) * vehicleSpeed;
      
      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) {
          cmd_vel.twist.linear.x = 0;
          cmd_vel.twist.linear.y = 0;
        } else {
          cmd_vel.twist.linear.x = cos(worldYaw) * mapSpeedX + sin(worldYaw) * mapSpeedY;
          cmd_vel.twist.linear.y = -sin(worldYaw) * mapSpeedX + cos(worldYaw) * mapSpeedY;
        }
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }
    else
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = 0;
      pubSpeed.publish(cmd_vel);
    }

    syncPackages();
    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "map_link", "world"));

    status = ros::ok();
    rate.sleep();
  
  }

  return 0;
}
