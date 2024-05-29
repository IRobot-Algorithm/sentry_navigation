#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Bool.h>
#include<pcl/io/pcd_io.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

bool color_init = false;
bool color_info;

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
                // for (int i = 0; i < 5; i++)
                // {
                //     point.z = 0.3 * i;
                //     cloud->push_back(point);
                // }
                point.z = 0.3;
                cloud->push_back(std::move(point));
                // point.z = 1.0;
                // cloud->push_back(std::move(point));
            }
        }
    }
}

void ColorInfoCallBack(const std_msgs::Bool::ConstPtr &msg_in)
{
    if (color_init)
        return;

    color_info = msg_in->data;
    color_init = true;
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
    ros::Publisher obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("static_obstacles", 1);
    ros::Publisher added_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/added_obstacles", 1);
    ros::Subscriber color_info_sub = nh.subscribe<std_msgs::Bool>("/color_info", 1, ColorInfoCallBack);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.6f, 0.6f, 0.6f);
    sor.filter(*filtered_cloud);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 output_filtered_cloud;
    pcl::toROSMsg(*filtered_cloud, output_filtered_cloud);
    output_filtered_cloud.header.frame_id = "map";

    ros::Rate loop_rate(0.2);
    while (ros::ok())
    {
        obstacle_pub.publish(output_cloud);
        added_obstacle_pub.publish(output_filtered_cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}