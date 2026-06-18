#!/usr/bin/env python
"""Preflight checks for the shared-map LIMO cluster."""

from __future__ import print_function

import math
import sys

import rosgraph
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


ROBOTS = ("robot1", "robot2", "robot3")


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


def valid_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(finite(v) for v in values):
        return False
    norm = math.sqrt(sum(v * v for v in values))
    return 0.5 <= norm <= 1.5


def stamp_age(stamp):
    if stamp is None or stamp.is_zero():
        return None
    return (rospy.Time.now() - stamp).to_sec()


def check_fresh(result, label, stamp, max_age, stale_is_failure=True):
    age = stamp_age(stamp)
    if age is None:
        result.warn("%s has zero timestamp" % label)
        return True
    if age > max_age:
        text = "%s stale: age %.2fs > %.2fs" % (label, age, max_age)
        if stale_is_failure:
            result.fail(text)
        else:
            result.warn(text)
        return False
    if age < -0.2:
        result.fail("%s from future: age %.2fs; check clock sync" % (label, age))
        return False
    return True


class CheckResult(object):
    def __init__(self):
        self.failures = 0
        self.warnings = 0

    def ok(self, text):
        print("[ OK ] " + text)

    def warn(self, text):
        self.warnings += 1
        print("[WARN] " + text)

    def fail(self, text):
        self.failures += 1
        print("[FAIL] " + text)


def wait_msg(topic, msg_type, timeout):
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        return None


def get_publishers():
    master = rosgraph.Master("/cluster_system_check")
    publishers, _, _ = master.getSystemState()
    return dict((topic, nodes) for topic, nodes in publishers)


def check_topic_publishers(result, publishers, topic, required=True,
                           max_publishers=None):
    nodes = publishers.get(topic, [])
    if not nodes:
        if required:
            result.fail("%s has no publisher" % topic)
        else:
            result.warn("%s has no publisher" % topic)
        return
    if max_publishers is not None and len(nodes) > max_publishers:
        result.fail("%s has %d publishers: %s" %
                    (topic, len(nodes), ", ".join(nodes)))
        return
    result.ok("%s publishers: %s" % (topic, ", ".join(nodes)))


def check_robot_messages(result, robot, timeout, max_age):
    scan_topic = "/" + robot + "/scan"
    odom_topic = "/" + robot + "/odom"
    pose_topic = "/" + robot + "/amcl_pose"

    scan = wait_msg(scan_topic, LaserScan, timeout)
    if scan is None:
        result.fail("%s no LaserScan within %.1fs" % (scan_topic, timeout))
    elif not scan.ranges:
        result.fail("%s empty ranges" % scan_topic)
    elif scan.header.frame_id != robot + "/base_laser":
        result.warn("%s frame_id=%s, expected %s/base_laser" %
                    (scan_topic, scan.header.frame_id, robot))
    else:
        check_fresh(result, scan_topic, scan.header.stamp, max_age)
        result.ok("%s received, frame=%s" % (scan_topic, scan.header.frame_id))

    odom = wait_msg(odom_topic, Odometry, timeout)
    if odom is None:
        result.fail("%s no Odometry within %.1fs" % (odom_topic, timeout))
    elif not valid_quaternion(odom.pose.pose.orientation):
        result.fail("%s invalid pose quaternion" % odom_topic)
    else:
        check_fresh(result, odom_topic, odom.header.stamp, max_age)
        result.ok("%s received" % odom_topic)

    pose = wait_msg(pose_topic, PoseWithCovarianceStamped, timeout)
    if pose is None:
        result.fail("%s no AMCL pose within %.1fs" % (pose_topic, timeout))
    else:
        p = pose.pose.pose.position
        q = pose.pose.pose.orientation
        if not (finite(p.x) and finite(p.y) and valid_quaternion(q)):
            result.fail("%s contains NaN/invalid quaternion" % pose_topic)
        else:
            # /amcl_pose is diagnostic. AMCL may publish it sparsely while the
            # robot is still, so stale pose messages are warnings. The hard
            # gating check for controllers is the live map -> base_link TF.
            check_fresh(result, pose_topic, pose.header.stamp, max_age,
                        stale_is_failure=False)
            result.ok("%s x=%.2f y=%.2f" % (pose_topic, p.x, p.y))


def check_tf(result, listener, robot, timeout, max_age):
    target = robot + "/base_link"
    try:
        listener.waitForTransform("map", target, rospy.Time(0),
                                  rospy.Duration(timeout))
        latest = listener.getLatestCommonTime("map", target)
        check_fresh(result, "TF map -> %s" % target, latest, max_age)
        trans, rot = listener.lookupTransform("map", target, rospy.Time(0))
        if not all(finite(v) for v in trans) or not all(finite(v) for v in rot):
            result.fail("TF map -> %s contains invalid values" % target)
        else:
            result.ok("TF map -> %s x=%.2f y=%.2f" %
                      (target, trans[0], trans[1]))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException) as exc:
        result.fail("TF map -> %s unavailable: %s" % (target, exc))


def check_control_conflicts(result, publishers):
    for topic in ("/robot1/cmd_vel", "/robot2/cmd_vel",
                  "/robot3/cmd_vel", "/robot2/cmd_vel_raw",
                  "/robot3/cmd_vel_raw"):
        check_topic_publishers(result, publishers, topic,
                               required=False, max_publishers=1)

    map_nodes = []
    nav_nodes = []
    for nodes in publishers.values():
        for node in nodes:
            if node.startswith("/map_follower_robot"):
                map_nodes.append(node)
            if "/move_base" in node:
                nav_nodes.append(node)
    if map_nodes and nav_nodes:
        result.fail("map_follower and move_base are both publishing/running: "
                    "%s ; %s" %
                    (", ".join(sorted(set(map_nodes))),
                     ", ".join(sorted(set(nav_nodes)))))


def main():
    rospy.init_node("cluster_system_check", anonymous=True)
    timeout = rospy.get_param("~timeout", 2.0)
    max_age = rospy.get_param("~max_age", 2.0)
    result = CheckResult()
    publishers = get_publishers()

    print("== Cluster System Check ==")
    for robot in ROBOTS:
        print("-- %s --" % robot)
        check_robot_messages(result, robot, timeout, max_age)

    listener = tf.TransformListener()
    rospy.sleep(0.3)
    for robot in ROBOTS:
        check_tf(result, listener, robot, timeout, max_age)

    print("-- command publishers --")
    check_control_conflicts(result, publishers)

    if result.failures:
        print("RESULT: FAIL (%d failures, %d warnings)" %
              (result.failures, result.warnings))
        return 1
    if result.warnings:
        print("RESULT: PASS WITH WARNINGS (%d warnings)" % result.warnings)
        return 0
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
