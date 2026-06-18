#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def make_pose(frame, x, y, yaw, covariance_xy, covariance_yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
    msg.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
    msg.pose.covariance = [0.0] * 36
    msg.pose.covariance[0] = float(covariance_xy)
    msg.pose.covariance[7] = float(covariance_xy)
    msg.pose.covariance[35] = float(covariance_yaw)
    return msg


def main():
    rospy.init_node("cluster_initial_pose_setter", anonymous=False)
    frame = rospy.get_param("~map_frame", "map")
    delay = float(rospy.get_param("~publish_delay", 2.0))
    count = int(rospy.get_param("~publish_count", 5))
    period = float(rospy.get_param("~publish_period", 0.3))
    covariance_xy = float(rospy.get_param("~covariance_xy", 0.25))
    covariance_yaw = float(rospy.get_param("~covariance_yaw", 0.068))
    robots = rospy.get_param("~robots", {})

    pubs = {}
    for robot in sorted(robots.keys()):
        pubs[robot] = rospy.Publisher("/%s/initialpose" % robot,
                                      PoseWithCovarianceStamped,
                                      queue_size=1,
                                      latch=True)

    rospy.sleep(delay)
    rate = rospy.Rate(1.0 / max(period, 0.05))
    for _ in range(max(count, 1)):
        for robot, pose in robots.items():
            msg = make_pose(frame,
                            pose.get("x", 0.0),
                            pose.get("y", 0.0),
                            pose.get("yaw", 0.0),
                            covariance_xy,
                            covariance_yaw)
            pubs[robot].publish(msg)
            rospy.loginfo_throttle(
                1.0,
                "Initial pose %s: x=%.2f y=%.2f yaw=%.2f",
                robot,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                pose.get("yaw", 0.0))
        rate.sleep()

    rospy.loginfo("Initial poses published. Exiting.")


if __name__ == "__main__":
    main()
