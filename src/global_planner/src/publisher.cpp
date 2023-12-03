#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
using namespace std;

geometry_msgs::PoseStamped way_goal;
geometry_msgs::PointStamped way_point;
geometry_msgs::PoseStamped rviz_goal;

ros::Publisher pubrviz_Goal;

void pathHandler(const nav_msgs::Path::ConstPtr& path)
{
	int PathLength;
	PathLength = path->poses.size();
	if(PathLength!=0)
	{
	vector<geometry_msgs::PoseStamped>::const_iterator it = path->poses.begin();
	if(PathLength > 51)//81
	{
		it = path->poses.begin()+49;//79
	}
	else
	{
		it = path->poses.end()-1;
	}
		way_goal = *it;
		way_goal.header.stamp = ros::Time().now();
		way_goal.header.frame_id = "map";
	}
}
void SimpleGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& sim_goal)
{
	rviz_goal.header.stamp = ros::Time().now();
    rviz_goal.header.frame_id = "map";
	rviz_goal.pose = sim_goal->pose;
}

int main(int argc, char **argv)
 {
 	ros::init(argc,argv,"publisher_goal");//ROS节点初始化，节点名称
 	ros::NodeHandle n;//创建节点句柄

	//创建订阅者
	// ros::Subscriber subPath = n.subscribe<nav_msgs::Path> ("/global_planner/planner/plan", 5, pathHandler);
	ros::Subscriber subPath = n.subscribe<nav_msgs::Path> ("/global_planner/planner/plan", 5, pathHandler);
 	
	ros::Subscriber subSimple_Goal = n.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 5, SimpleGoalHandler);
 	//创建发布者，定义话题名

 	pubrviz_Goal = n.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 10);//asker 是话题名，1000是队列大小

 	ros::Publisher pubGoal = n.advertise<geometry_msgs::PointStamped>("/way_point", 10);//asker 是话题名，1000是队列大小

 	ros::Rate  loop_rate(10);//设置循环的频率，后面会用 500
 	int count = 0;
 	while(ros::ok())
 	{
        // geometry_msgs::PoseStamped goal;
        // goal.header.frame_id = "map";
        // goal.header.stamp = ros::Time::now();
        // goal.pose.position.x = 10.0;
        // goal.pose.position.y = 0.0;
        // goal.pose.position.z = 0.0;
        // goal.pose.orientation.x = 0.0;
        // goal.pose.orientation.y = 0.0;
        // goal.pose.orientation.z = 0.0;
        // goal.pose.orientation.w = 1.0;
		
		rviz_goal.header.stamp = ros::Time().now();
    	rviz_goal.header.frame_id = "map";
        pubrviz_Goal.publish(rviz_goal);

		way_point.header.stamp = ros::Time().now();
    	way_point.header.frame_id = "map";
		way_point.point.x=way_goal.pose.position.x;
		way_point.point.y=way_goal.pose.position.y;
		way_point.point.z=way_goal.pose.position.z;
		pubGoal.publish(way_point);
		ros::spinOnce();//等待一次
		loop_rate.sleep();
	}
	return 0;
}
