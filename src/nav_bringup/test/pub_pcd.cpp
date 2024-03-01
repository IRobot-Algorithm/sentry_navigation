#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_pointcloud_node");
    ros::NodeHandle nh;

    // 创建一个发布者来发布 PointCloud2 消息
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // 从PCD文件加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/path/to/your/pcd/file.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    // 将 PointCloud 转换为 PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map"; // 设置点云的坐标系

    ros::Rate loop_rate(2); // 设置发布频率为每秒2次

    while (ros::ok()) {
        cloud_msg.header.stamp = ros::Time::now(); // 更新时间戳
        pub.publish(cloud_msg); // 发布点云消息
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
