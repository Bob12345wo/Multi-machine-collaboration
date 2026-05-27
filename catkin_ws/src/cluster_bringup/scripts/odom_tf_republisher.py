#!/usr/bin/env python
"""
Republish Odometry messages as namespaced TF frames.

The stock LIMO driver publishes generic TF frame names such as odom and
base_link. In a shared ROS master with multiple robots, those frames collide.
This helper creates robot-specific TF frames from /robotX/odom:

  /robot1/odom -> robot1/odom -> robot1/base_link
  /robot2/odom -> robot2/odom -> robot2/base_link

It is a bridge for multi-robot formation and localization. When AMCL is added,
AMCL should publish map -> robotX/odom, while this node keeps publishing
robotX/odom -> robotX/base_link.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfRepublisher(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic')
        self.odom_frame = rospy.get_param('~odom_frame')
        self.base_frame = rospy.get_param('~base_frame')
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.callback,
                                    queue_size=10)
        rospy.loginfo("Republishing %s as TF %s -> %s",
                      self.odom_topic, self.odom_frame, self.base_frame)

    def callback(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(tf_msg)


if __name__ == '__main__':
    rospy.init_node('odom_tf_republisher')
    OdomTfRepublisher()
    rospy.spin()
