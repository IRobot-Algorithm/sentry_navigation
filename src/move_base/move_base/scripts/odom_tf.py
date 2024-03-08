#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

class OdomAndTfPublisher:
    def __init__(self):
        rospy.init_node('odom_and_tf_publisher')
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/ep/odom', Odometry, self.odom_callback)
        rospy.sleep(1.0)  # Give the tf_listener time to fill its buffer

    def odom_callback(self, msg):
        self.tf_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            msg.header.stamp,
            "base_link",
            "odom"
        )
        self.publish_transformed_odom(msg)

    def publish_transformed_odom(self, original_odom_msg):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            linear_vel = self.transform_linear_velocity(original_odom_msg.twist.twist.linear, original_odom_msg.header.frame_id, 'map')

            new_odom_msg = Odometry()
            new_odom_msg.header.stamp = rospy.Time.now()
            new_odom_msg.header.frame_id = "map"
            new_odom_msg.child_frame_id = "base_link"

            # Set the position based on map to base_link transformation
            new_odom_msg.pose.pose.position.x = trans[0]
            new_odom_msg.pose.pose.position.y = trans[1]
            new_odom_msg.pose.pose.position.z = trans[2]
            new_odom_msg.pose.pose.orientation.x = rot[0]
            new_odom_msg.pose.pose.orientation.y = rot[1]
            new_odom_msg.pose.pose.orientation.z = rot[2]
            new_odom_msg.pose.pose.orientation.w = rot[3]

            # Set the transformed linear velocity
            if linear_vel is not None:
                new_odom_msg.twist.twist.linear.x = linear_vel.vector.x
                new_odom_msg.twist.twist.linear.y = linear_vel.vector.y
                new_odom_msg.twist.twist.linear.z = linear_vel.vector.z

            self.odom_pub.publish(new_odom_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to transform odom: %s" % str(e))

    def transform_linear_velocity(self, linear, from_frame, to_frame):
        vector3_stamped = Vector3Stamped()
        vector3_stamped.header.stamp = rospy.Time(0)
        vector3_stamped.header.frame_id = from_frame
        vector3_stamped.vector = linear

        try:
            transformed_vector = self.tf_listener.transformVector3(to_frame, vector3_stamped)
            return transformed_vector
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to transform linear velocity: %s" % str(e))
            return None

if __name__ == '__main__':
    odom_and_tf_publisher = OdomAndTfPublisher()
    rospy.spin()
