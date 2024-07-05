#!/usr/bin/python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import std_msgs.msg

def read_pcd(file_path):
    # Load the PCD file using open3d
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def convert_pcd_to_pointcloud2(pcd):
    # Convert open3d point cloud to PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # Change this to the frame you are using
    
    points = np.asarray(pcd.points)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    cloud_msg = point_cloud2.create_cloud(header, fields, points)
    return cloud_msg

if __name__ == '__main__':
    rospy.init_node('pcd_publisher')
    pub = rospy.Publisher('pcd_topic', PointCloud2, queue_size=10)
    
    file_path = '/home/niuoruo/workspace/sentry/ws/sentry_navigation/src/fast_gicp/data/map.pcd'  # Change this to your PCD file path
    pcd = read_pcd(file_path)
    cloud_msg = convert_pcd_to_pointcloud2(pcd)
    
    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        pub.publish(cloud_msg)
        rate.sleep()
