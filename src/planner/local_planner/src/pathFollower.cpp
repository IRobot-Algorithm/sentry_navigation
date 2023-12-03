#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
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
double dirDiffThre = 0.3;
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
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;//2.0

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
bool safetyStop = false;

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
float vehicleSpeed_X = 0;
float vehicleSpeed_Y = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd_x = true;
double switchTime_x = 0;
bool navFwd_y = true;
double switchTime_y = 0;

//zbh
float goalX = 10.0;
float goalY = 10.0;
float vyaw = 0.0;
double imu_roll,imu_pitch,imu_yaw;


nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

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
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

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
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imu)
{
  // vyaw = imu->angular_velocity.z;
  geometry_msgs::Quaternion geoQuat_imu;
  geoQuat_imu.x = imu->orientation.x;
  geoQuat_imu.y = imu->orientation.y;
  geoQuat_imu.z = imu->orientation.z;
  geoQuat_imu.w = imu->orientation.w;
  tf::Matrix3x3(tf::Quaternion(geoQuat_imu.x, geoQuat_imu.y, geoQuat_imu.z, geoQuat_imu.w)).getRPY(imu_roll, imu_pitch, imu_yaw);
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
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
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
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/Odometry", 1, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/nav_path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Bool> ("/stop", 5, stopHandler);

  //订阅航点（位姿）
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  ros::Subscriber subIMU = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 5, imuHandler);

  //发布导航状态
  ros::Publisher pubnavState = nh.advertise<std_msgs::Bool> ("/nav_state", 5);
  std_msgs::Bool navpoint_stateMsgs;

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "map";//"vehicle"

  //定义tf监听odom->base_link的位姿
  tf::TransformListener tf_listener;
  tf::StampedTransform transform_map2baselink;

  double tf_roll, tf_pitch, tf_yaw;
  geometry_msgs::Quaternion geoQuat_tf;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(200);//100
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    // try{
    //   ros::Time now = ros::Time::now();
    //   tf_listener.waitForTransform("odom", "base_link",now, ros::Duration(0.01));
    //   tf_listener.lookupTransform("odom", "base_link",now, transform_map2baselink);
    //   }
    //   catch (tf::TransformException &ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(0.1).sleep();
    //   continue;
    //   }
    geoQuat_tf.x = transform_map2baselink.getRotation().getX();
    geoQuat_tf.y = transform_map2baselink.getRotation().getY();
    geoQuat_tf.z = transform_map2baselink.getRotation().getZ();
    geoQuat_tf.w = transform_map2baselink.getRotation().getW();
    // transform.getOrigin().getX();
    // transform.getRotation().getX();
    tf::Matrix3x3(tf::Quaternion(geoQuat_tf.x, geoQuat_tf.y, geoQuat_tf.z, geoQuat_tf.w)).getRPY(tf_roll, tf_pitch, tf_yaw);


    bool navpoint_state = false;
    float nav_dis = 0.0;
    nav_dis = sqrt((goalX-vehicleX)*(goalX-vehicleX)+(goalY-vehicleY)*(goalY-vehicleY));
    float nav_disX = fabs(goalX-vehicleX);
    float nav_disY = fabs(goalY-vehicleY);
    if(nav_dis<0.3)
    {
      navpoint_state = true;
    }
    else
    {
      navpoint_state = false;
    }
    navpoint_stateMsgs.data = navpoint_state;
    pubnavState.publish(navpoint_stateMsgs);

    if(!navpoint_state)
    {
    if(nav_dis<=0.6)
    {
      maxSpeed = 0.5;
    }
    else
    {
      maxSpeed = 0.8;//1.1
    }
      
    if (pathInit) {
      float vehicleXRel = (vehicleX - vehicleXRec);
      float vehicleYRel = (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
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
      // float mappathX = cos(vehicleYawRec)*path.poses[pathPointID].pose.position.x - sin(vehicleYawRec)*path.poses[pathPointID].pose.position.y;
      // float mappathY = sin(vehicleYawRec)*path.poses[pathPointID].pose.position.x + cos(vehicleYawRec)*path.poses[pathPointID].pose.position.y;

      float pathDir_x = atan2(disY, disX);
      float pathDir_y = atan2(disY, disX) - PI/2;
      if (pathDir_y > PI) pathDir_y -= 2 * PI;
      else if (pathDir_y < -PI) pathDir_y += 2 * PI;


      // float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      // if (dirDiff > PI) dirDiff -= 2 * PI;
      // else if (dirDiff < -PI) dirDiff += 2 * PI;
      // if (dirDiff > PI) dirDiff -= 2 * PI;
      // else if (dirDiff < -PI) dirDiff += 2 * PI;

      // if (twoWayDrive) {
      //   double time = ros::Time::now().toSec();
      //   if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
      //     navFwd = false;
      //     switchTime = time;
      //   } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
      //     navFwd = true;
      //     switchTime = time;
      //   }
      // }

      // float joySpeed2 = maxSpeed * joySpeed;
      // if (!navFwd) {
      //   dirDiff += PI;
      //   if (dirDiff > PI) dirDiff -= 2 * PI;
      //   joySpeed2 *= -1;
      // }

      // if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      // else vehicleYawRate = -yawRateGain * dirDiff;


      float dirDiff_x = - pathDir_x;
      if (dirDiff_x > PI) dirDiff_x -= 2 * PI;
      else if (dirDiff_x < -PI) dirDiff_x += 2 * PI;
      if (dirDiff_x > PI) dirDiff_x -= 2 * PI;
      else if (dirDiff_x < -PI) dirDiff_x += 2 * PI;
      if (twoWayDrive) {
        double time_x = ros::Time::now().toSec();
        if (fabs(dirDiff_x) > PI / 2 && navFwd_x && time_x - switchTime_x > switchTimeThre) {
          navFwd_x = false;
          switchTime_x = time_x;
        } else if (fabs(dirDiff_x) < PI / 2 && !navFwd_x && time_x - switchTime_x > switchTimeThre) {
          navFwd_x = true;
          switchTime_x = time_x;
        }
      }
      float joySpeed2_x = maxSpeed * joySpeed;
      if (!navFwd_x) {
        dirDiff_x += PI;
        if (dirDiff_x > PI) dirDiff_x -= 2 * PI;
        joySpeed2_x *= -1;
      }


      float dirDiff_y = - pathDir_y;
      if (dirDiff_y > PI) dirDiff_y -= 2 * PI;
      else if (dirDiff_y < -PI) dirDiff_y += 2 * PI;
      if (dirDiff_y > PI) dirDiff_y -= 2 * PI;
      else if (dirDiff_y < -PI) dirDiff_y += 2 * PI;
      if (twoWayDrive) {
        double time_y = ros::Time::now().toSec();
        if (fabs(dirDiff_y) > PI / 2 && navFwd_y && time_y - switchTime_y > switchTimeThre) {
          navFwd_y = false;
          switchTime_y = time_y;
        } else if (fabs(dirDiff_y) < PI / 2 && !navFwd_y && time_y - switchTime_y > switchTimeThre) {
          navFwd_y = true;
          switchTime_y = time_y;
        }
      }
      float joySpeed2_y = maxSpeed * joySpeed;
      if (!navFwd_y) {
        dirDiff_y += PI;
        if (dirDiff_y > PI) dirDiff_y -= 2 * PI;
        joySpeed2_y *= -1;
      }

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (pathSize <= 1) {
        joySpeed2_x = 0;
        joySpeed2_y = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2_x *= endDis / slowDwnDisThre;
        joySpeed2_y *= endDis / slowDwnDisThre;
      }

      float joySpeed3_x = joySpeed2_x;
      float joySpeed3_y = joySpeed2_y;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) 
      {
        joySpeed3_x *= slowRate1;
        joySpeed3_y *= slowRate1;
      }
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) 
      {
        joySpeed3_x *= slowRate2;
        joySpeed3_y *= slowRate2;
      }

      
      if ( fabs(disX) > stopDisThre) {
        if (vehicleSpeed_X < joySpeed3_x) vehicleSpeed_X += maxAccel / 100.0;
        else if (vehicleSpeed_X > joySpeed3_x) vehicleSpeed_X -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed_X > 0) vehicleSpeed_X -= maxAccel / 100.0;
        else if (vehicleSpeed_X < 0) vehicleSpeed_X += maxAccel / 100.0;
      }
      if ( fabs(disY) > stopDisThre) {
        if (vehicleSpeed_Y < joySpeed3_y) vehicleSpeed_Y += maxAccel / 100.0;
        else if (vehicleSpeed_Y > joySpeed3_y) vehicleSpeed_Y -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed_Y > 0) vehicleSpeed_Y -= maxAccel / 100.0;
        else if (vehicleSpeed_Y < 0) vehicleSpeed_Y += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed_X = 0;
        vehicleSpeed_Y = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop) 
      {
      vehicleSpeed_X = 0;
      vehicleSpeed_Y = 0;
      }
      // if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        // if (fabs(vehicleSpeed_X) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        // else cmd_vel.twist.linear.x = vehicleSpeed_X;
        // if (fabs(vehicleSpeed_Y) <= maxAccel / 100.0) cmd_vel.twist.linear.y = 0;
        // else cmd_vel.twist.linear.y = vehicleSpeed_Y;

        // if(nav_disX<0.15)
        // {
        //   vehicleSpeed_X = 0.0;
        // }
        // if(nav_disY<0.15)
        // {
        //   vehicleSpeed_Y = 0.0;
        // }
        
        float vehicleSpeed_X2 = vehicleSpeed_X;
        float vehicleSpeed_Y2 = vehicleSpeed_Y;

        if(fabs(disX)>fabs(disY))
        {
          if(vehicleSpeed_Y!=0 && disX!=0)
          {
          vehicleSpeed_Y2 = (vehicleSpeed_Y/fabs(vehicleSpeed_Y))*fabs(vehicleSpeed_X)*(fabs(disY)/fabs(disX));
          }
        }
        else if(fabs(disX)<fabs(disY))
        {
          if(vehicleSpeed_X!=0 && disY!=0)
          {
          vehicleSpeed_X2 = (vehicleSpeed_X/fabs(vehicleSpeed_X))*fabs(vehicleSpeed_Y)*(fabs(disX)/fabs(disY));
          }
        }

        float vehicleSpeed_X3,vehicleSpeed_Y3;

        // 控制变量法测试
        // vehicleSpeed_X = 1.0;
        // vehicleSpeed_Y = 0.0;
        // float vehicleSpeed_X3 = cos(vehicleYaw)*vehicleSpeed_X2 + sin(vehicleYaw)*vehicleSpeed_Y2;
        // float vehicleSpeed_Y3 = - sin(vehicleYaw)*vehicleSpeed_X2 + cos(vehicleYaw)*vehicleSpeed_Y2;
        // float vehicleSpeed_X3 = cos(imu_yaw)*vehicleSpeed_X2 + sin(imu_yaw)*vehicleSpeed_Y2;
        // float vehicleSpeed_Y3 = - sin(imu_yaw)*vehicleSpeed_X2 + cos(imu_yaw)*vehicleSpeed_Y2;

        // vehicleYaw = vehicleYaw + 0.05 * vyaw;
        // vehicleSpeed_X3 = cos(vehicleYaw) * 0.5 + sin(vehicleYaw) * 0.0;
        // vehicleSpeed_Y3 = -sin(vehicleYaw) * 0.5 + cos(vehicleYaw) * 0.0;
        // vehicleSpeed_X3 = cos(imu_yaw) * 0.5 + sin(imu_yaw) * 0.0;
        // vehicleSpeed_Y3 = -sin(imu_yaw) * 0.5 + cos(imu_yaw) * 0.0;
        // std::cout<<imu_yaw<<std::endl;

        // std::cout<<"tf_yaw:"<<tf_yaw<<endl;
        // std::cout<<"tf_roll:"<<tf_roll<<endl;
        // std::cout<<"tf_pitch:"<<tf_pitch<<endl;
        // std::cout<<"geoQuat_tf.x:"<<geoQuat_tf.x<<endl;
        // std::cout<<"geoQuat_tf.y:"<<geoQuat_tf.y<<endl;
        // std::cout<<"geoQuat_tf.z:"<<geoQuat_tf.z<<endl;
        // std::cout<<"geoQuat_tf.w:"<<geoQuat_tf.w<<endl;

        // vehicleSpeed_X3 = cos(tf_yaw) * 1.0 + sin(tf_yaw) * 0.0;
        // vehicleSpeed_Y3 = -sin(tf_yaw) * 1.0 + cos(tf_yaw) * 0.0;
        vehicleSpeed_X3 = cos(vehicleYaw) * vehicleSpeed_X2 + sin(vehicleYaw) * vehicleSpeed_Y2;
        vehicleSpeed_Y3 = -sin(vehicleYaw) * vehicleSpeed_X2 + cos(vehicleYaw) * vehicleSpeed_Y2;

        cmd_vel.twist.linear.x = vehicleSpeed_X3;
        cmd_vel.twist.linear.y = vehicleSpeed_Y3;
        // cmd_vel.twist.angular.z = vehicleYawRate;
        cmd_vel.twist.angular.z = 0;
        pubSpeed.publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }
    }
    }
    else if(navpoint_state)
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      // cmd_vel.twist.angular.z = vehicleYawRate;
      cmd_vel.twist.angular.z = 0;
      pubSpeed.publish(cmd_vel);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
