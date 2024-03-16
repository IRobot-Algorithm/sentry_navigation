#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <pcl/point_types.h>

/**
 * 将占据栅格数据转为pointcloud点云格式数据
 */
void OccupancyGridToPointCloud(const nav_msgs::OccupancyGrid &map_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // 获取地图的分辨率
    double resolution = map_msg.info.resolution;

    // 获取地图的原点坐标
    double origin_x = map_msg.info.origin.position.x;
    double origin_y = map_msg.info.origin.position.y;

    // 获取地图的宽度和高度
    int width = map_msg.info.width;
    int height = map_msg.info.height;

    // 遍历地图中的每个网格单元
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 获取当前网格单元的占用情况
            int index = y * width + x;
            int occupancy = map_msg.data[index];

            // 如果当前网格单元被占用，则将其对应的坐标添加到点云中
            if (occupancy > 0) {
                // 计算当前网格单元的坐标
                double pos_x = origin_x + x * resolution;
                double pos_y = origin_y + y * resolution;

                // 创建一个三维点并添加到点云中
                pcl::PointXYZ point;
                point.x = pos_x;
                point.y = pos_y;
                point.z = 0.0;
                cloud->push_back(point);
            }
        }
    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "obstacleGenerating");
    ros::NodeHandle nh;

    if (!ros::service::waitForService("static_map", ros::Duration(10.0))) {
        ROS_ERROR("Failed to wait for service static_map");
        return 1;
    }

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");
    ros::Publisher obstacle_pub = nh.advertise<sensor_msgs::PointCloud2> ("static_obstacles", 1);
    
    nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid map;

    if (client.call(srv)) {
        ROS_INFO("Received map");
        map = srv.response.map;
    } else {
        ROS_ERROR("Failed to call service static_map");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    OccupancyGridToPointCloud(map, cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
 
    ros::Rate loop_rate(0.2);
    while (ros::ok())
    {
        obstacle_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}