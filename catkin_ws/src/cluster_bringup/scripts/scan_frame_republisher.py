#!/usr/bin/env python
"""
Republish LaserScan with a robot-specific frame_id.

Many single-robot launch files publish scans with frame_id=base_laser. In a
shared ROS master, two robots using the same frame collide. This node keeps the
scan data unchanged and rewrites only the header frame.
"""

import rospy
from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(object):
    def __init__(self):
        self.input_scan = rospy.get_param('~input_scan')
        self.output_scan = rospy.get_param('~output_scan')
        self.frame_id = rospy.get_param('~frame_id')
        self.pub = rospy.Publisher(self.output_scan, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input_scan, LaserScan, self.callback,
                                    queue_size=10)
        rospy.loginfo("Republishing %s to %s with frame_id=%s",
                      self.input_scan, self.output_scan, self.frame_id)

    def callback(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('scan_frame_republisher')
    ScanFrameRepublisher()
    rospy.spin()
