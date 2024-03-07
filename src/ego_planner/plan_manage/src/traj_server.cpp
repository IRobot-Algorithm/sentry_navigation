#include "bspline_opt/uniform_bspline.h"
#include "MPC.hpp"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "ego_planner/Bspline.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
//#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "time.h"
#include <mutex>

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
#define N 15
// #define sim

ros::Publisher vel_cmd_pub;
Eigen::MatrixXd u_k;
std::mutex mtx;
std::mutex callback_mutex;

//quadrotor_msgs::PositionCommand cmd;
#ifdef sim
geometry_msgs::Twist cmd;
#else
geometry_msgs::TwistStamped cmd;
#endif
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool is_orientation_init = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_,time_s,time_e;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
double roll,pitch,yaw;
geometry_msgs::PoseStamped pose_cur;
tf::Quaternion quat;
std_msgs::UInt8 is_adjust_pose;
std_msgs::UInt8 dir;

// enum DIRECTION {POSITIVE=0,NEGATIVE=1};

// yaw control
double t_step;

std_msgs::UInt8 stop_command;

////time record
clock_t start_clock,end_clock;
double duration;

void mpcStopCallback();

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }


  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);


  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  //ROS_INFO("Receive b-spline trajectory!");

  receive_traj_ = true;

  if (msg->pos_pts[0].z > 50)
    mpcStopCallback();
}

void poseCallback(geometry_msgs::PoseStampedConstPtr msg)
{
    pose_cur = *msg;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(std_msgs::UInt8ConstPtr msg)
{
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::UInt8ConstPtr& msg)
{
    dir = *msg;
}

void MPC_calculate(double &t_cur)
{
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::Vector3d pos_r,pos_r_1,pos_final,v_r_1,v_r_2,X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r,x_r_1,x_r_2;
    double v_linear_1,w;
    double t_k,t_k_1;

    //ROS_INFO("Run to here!");

    //Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
    //Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
    Eigen::Vector3d vel_start = traj_[1].evaluateDeBoor(t_cur);
    double yaw_start = atan2(vel_start(1),vel_start(0));
    bool is_orientation_adjust=false;
    double orientation_adjust=0;
    pos_final = traj_[0].evaluateDeBoor(traj_duration_);

        is_orientation_init=true;
        for(int i=0;i<N;i++)
        {

            t_k = t_cur+i*t_step;
            t_k_1 = t_cur+(i+1)*t_step;

            pos_r = traj_[0].evaluateDeBoor(t_k);
            pos_r_1 = traj_[0].evaluateDeBoor(t_k_1);

            x_r(0) = pos_r(0);
            x_r(1) = pos_r(1);

            v_r_1 = traj_[1].evaluateDeBoor(t_k);
            v_r_2 = traj_[1].evaluateDeBoor(t_k_1);
            v_r_1(2)=0;
            v_r_2(2)=0;
            v_linear_1 = v_r_1.norm();
            if((t_k-traj_duration_)>=0)
            {
                x_r(2) = atan2((pos_r-pos_final)(1),(pos_r-pos_final)(0));
            }
            else
            {
                //x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
                x_r(2) = atan2(v_r_1(1),v_r_1(0));
            }



            double yaw1 = atan2(v_r_1(1),v_r_1(0));
            double yaw2 = atan2(v_r_2(1),v_r_2(0));

            if(abs(yaw2-yaw1)>PI)
            {

                is_orientation_adjust = true;
                if((yaw2-yaw1)<0)
                {
                    orientation_adjust = 2*PI;
                    w = (2*PI+(yaw2-yaw1))/t_step;
                }
                else
                {
                    w = ((yaw2-yaw1)-2*PI)/t_step;
                    orientation_adjust = -2*PI;
                }
            }
            else
            {
                w = (yaw2-yaw1)/t_step;
            }

            if(is_orientation_adjust==true)
            {
                //cout<<"orientation before adjust : "<< x_r(2)<<endl;
                x_r(2) +=orientation_adjust;
                //cout<<"orientation after adjust : "<< x_r(2)<<endl;
            }

            u_r(0) = v_linear_1;
            u_r(1) = w;
//                if(t_c>(tp-5*t_step))
//                {
//                    cout<<"Ur "<<i+1<<" : "<<endl<<u_r<<endl;
//                    cout<<"Xr "<<i+1<<" : "<<endl<<x_r<<endl;
//                }
            X_r.push_back(x_r);
            U_r.push_back(u_r);
        }

        //X_k(0) = odom_map.pose.pose.position.x - my_map.info.origin.position.x;
        //X_k(1) = odom_map.pose.pose.position.y - my_map.info.origin.position.y;
        X_k(0) = odom_pos_(0);
        X_k(1) = odom_pos_(1);
        if(yaw/X_r[0](2)<0&&abs(yaw)>(PI*5/6))
        {
            if(yaw<0)
            {
                X_k(2) = yaw + 2*PI;
            }
            else
            {
                X_k(2) = yaw - 2*PI;
            }
        }
        else
        {
            X_k(2) = yaw;
        }
        
        Eigen::MatrixXd x = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);
        cmd.twist.linear.x = x.col(0)(0);
        cmd.twist.angular.z = x.col(0)(1);
        vel_cmd_pub.publish(cmd);
        std::cout << "cmd.twist.angular.z" << cmd.twist.angular.z << std::endl;
        mtx.lock();
            u_k = x;
        mtx.unlock();

}

void stopCallback(std_msgs::UInt8ConstPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // if(dir.data==NEGATIVE)
    // {
    //     if(yaw>0)
    //     {
    //         yaw -= PI;
    //     }else if(yaw<0)
    //     {
    //         yaw += PI;
    //     }
    // }

}

void cmdCallback(const ros::TimerEvent &e)
{
    if (stop_command.data==1)
    {
#ifdef sim
        cmd.angular.z = 0;
        cmd.linear.x = 0;
#else
        cmd.twist.angular.z = 0;
        cmd.twist.linear.x = 0;
#endif
        vel_cmd_pub.publish(cmd);
        return;
    }

	  if (is_adjust_pose.data == 1)
        return;
    /* no publishing before receive traj_ */

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    ros::Time time = ros::Time::now();
    double dt = (time - time_s).toSec();

    int n = dt / t_step;
    std::cout << "n:" << n << std::endl;
    mtx.lock();
    if (n > -1e-5 && n < u_k.cols())
    {
#ifdef sim
      cmd.linear.x = u_k.col(n)(0);
      cmd.angular.z = u_k.col(n)(1);
#else
      cmd.twist.linear.x = u_k.col(n)(0);
      cmd.twist.angular.z = u_k.col(n)(1);
#endif
    }
    else
    {
#ifdef sim
      cmd.linear.x = 0;
      cmd.angular.z = 0;
#else
      cmd.twist.linear.x = 0;
      cmd.twist.angular.z = 0;
#endif
    }
    mtx.unlock();

    vel_cmd_pub.publish(cmd);
}

void mpcCallback(const ros::TimerEvent &e)
{
    std::lock_guard<std::mutex> lock(callback_mutex);

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    ros::Time time = ros::Time::now();
    double t_cur = (time - start_time_).toSec();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        ROS_INFO("MPC_Calculate!");
        start_clock = clock();
        MPC_calculate(t_cur);
        end_clock = clock();
        duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        ROS_INFO("Control times : %f ms",duration);
    }
    else if (t_cur >= traj_duration_)
    {
        mtx.lock();
        u_k.setZero();
        mtx.unlock();
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_s = time;

}


void mpcStopCallback()
{
    std::lock_guard<std::mutex> lock(callback_mutex);

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    ros::Time time = ros::Time::now();
    double t_cur = (time - start_time_).toSec();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        // ROS_INFO("MPC_Calculate!");
        // start_clock = clock();
        MPC_calculate(t_cur);
        // end_clock = clock();
        // duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        // ROS_INFO("Control times : %f ms",duration);
    }
    else if (t_cur >= traj_duration_)
    {
        mtx.lock();
        u_k.setZero();
        mtx.unlock();
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_s = time;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node("~");

  std::string cmd_topic,pose_topic;
  node.getParam("/ego_planner_node/fsm/pose_topic", pose_topic);
  node.getParam("/ego_planner_node/fsm/vel_topic", cmd_topic);

  ros::Subscriber bspline_sub = node.subscribe("/planning/bspline", 10, bsplineCallback);
  ros::Subscriber pose_sub = node.subscribe(pose_topic, 10, poseCallback);
  ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
  ros::Subscriber stop_sub = node.subscribe("/emergency_stop",10,stopCallback);
  ros::Subscriber adjust_yaw_sub = node.subscribe("/is_adjust_yaw",10,adjust_yaw_Callback);
  ros::Subscriber command_sub = node.subscribe("/direction",10,dirCallback);

  mpc_controller.MPC_init(node);
#ifdef sim
  vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
#else
  vel_cmd_pub = node.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 50);
  cmd.header.frame_id = "base_link";
#endif
  stop_command.data = 0;
//   dir.data = POSITIVE;
  t_step = 0.1;
  is_adjust_pose.data = 0;

//   ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer mpc_timer = node.createTimer(ros::Duration(0.03), mpcCallback);

//  nh.param("traj_server/time_forward", time_forward_, -1.0);
//  last_yaw_ = 0.0;
//  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin();

  return 0;
}