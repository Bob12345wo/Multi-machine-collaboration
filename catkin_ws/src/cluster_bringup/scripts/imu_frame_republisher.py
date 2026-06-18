#!/usr/bin/env python
"""
Republish IMU messages with a robot-specific frame_id.

The stock LIMO base node publishes IMU data with frame_id=imu_link. In a
multi-robot ROS master this frame collides across cars. This node keeps the IMU
data unchanged and rewrites only the header frame.
"""

from copy import deepcopy

import rospy
from sensor_msgs.msg import Imu


class ImuFrameRepublisher(object):
    def __init__(self):
        self.input_imu = rospy.get_param('~input_imu')
        self.output_imu = rospy.get_param('~output_imu')
        self.frame_id = rospy.get_param('~frame_id')
        self.max_rate = float(rospy.get_param('~max_rate', 30.0))
        self.min_period = (1.0 / self.max_rate
                           if self.max_rate > 0.0 else 0.0)
        self.last_publish_time = rospy.Time(0)
        self.pub = rospy.Publisher(self.output_imu, Imu, queue_size=1)
        self.sub = rospy.Subscriber(self.input_imu, Imu, self.callback,
                                    queue_size=1, tcp_nodelay=True)
        rospy.loginfo("Republishing %s to %s with frame_id=%s max_rate=%.1fHz",
                      self.input_imu, self.output_imu, self.frame_id,
                      self.max_rate)

    def callback(self, msg):
        now = rospy.Time.now()
        if self.min_period > 0.0 and self.last_publish_time.to_sec() > 0.0:
            elapsed = (now - self.last_publish_time).to_sec()
            if 0.0 <= elapsed < self.min_period:
                return
        self.last_publish_time = now

        out = Imu()
        out.header = deepcopy(msg.header)
        out.header.frame_id = self.frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('imu_frame_republisher')
    ImuFrameRepublisher()
    rospy.spin()
