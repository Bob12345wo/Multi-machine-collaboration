#!/usr/bin/env python
"""
Fake odometry publisher for simulation testing.
Subscribes to cmd_vel, integrates it, and publishes odometry.
Useful for testing cluster control without real hardware.
"""

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class FakeOdomPublisher:
    def __init__(self):
        self.ns = rospy.get_namespace().strip('/')
        rospy.loginfo("FakeOdomPublisher starting in namespace: %s", self.ns)

        # Load params
        self.initial_x = rospy.get_param('~initial_x', 0.0)
        self.initial_y = rospy.get_param('~initial_y', 0.0)
        self.initial_theta = rospy.get_param('~initial_theta', 0.0)
        self.rate = rospy.get_param('~rate', 50.0)

        # State
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = self.initial_theta
        self.vx = 0.0
        self.vz = 0.0
        self.last_time = rospy.Time.now()

        # Subscribers
        cmd_vel_topic = '/' + self.ns + '/cmd_vel' if self.ns else '/cmd_vel'
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Publishers
        odom_topic = '/' + self.ns + '/odom' if self.ns else '/odom'
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_callback)

        rospy.loginfo("FakeOdomPublisher ready. Publishing odom at %s", odom_topic)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vz = msg.angular.z

    def timer_callback(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.rate
        self.last_time = now

        # Integrate odometry
        self.theta += self.vz * dt
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = '/' + self.ns + '/odom' if self.ns else 'odom'
        odom.child_frame_id = '/' + self.ns + '/base_link' if self.ns else 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # quaternion from yaw
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*q)

        # Twist
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vz

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = '/' + self.ns + '/odom' if self.ns else 'odom'
        t.child_frame_id = '/' + self.ns + '/base_link' if self.ns else 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*q)

        self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('fake_odom_publisher')
    node = FakeOdomPublisher()
    rospy.spin()
