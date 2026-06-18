#!/usr/bin/env python
"""Log move_base goal/status/cmd flow for one follower robot."""

from __future__ import print_function

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist


STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class NavStatusMonitor(object):
    def __init__(self):
        robot = rospy.get_param("~robot", "robot2")
        self.robot = robot.strip("/")
        self.debug_period = rospy.get_param("~debug_period", 1.0)
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/" + self.robot + "/assigned_goal")
        self.status_topic = rospy.get_param(
            "~status_topic", "/" + self.robot + "/move_base/status")
        self.raw_cmd_topic = rospy.get_param(
            "~raw_cmd_topic", "/" + self.robot + "/cmd_vel_raw")
        self.final_cmd_topic = rospy.get_param(
            "~final_cmd_topic", "/" + self.robot + "/cmd_vel")

        self.last_goal = None
        self.last_goal_time = rospy.Time(0)
        self.status_text = "NO_STATUS"
        self.status_stamp = rospy.Time(0)
        self.last_raw_cmd = Twist()
        self.last_raw_cmd_time = rospy.Time(0)
        self.last_final_cmd = Twist()
        self.last_final_cmd_time = rospy.Time(0)

        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb,
                         queue_size=10)
        rospy.Subscriber(self.status_topic, GoalStatusArray, self.status_cb,
                         queue_size=10)
        rospy.Subscriber(self.raw_cmd_topic, Twist, self.raw_cmd_cb,
                         queue_size=10)
        rospy.Subscriber(self.final_cmd_topic, Twist, self.final_cmd_cb,
                         queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.debug_period),
                                 self.timer_cb)
        rospy.loginfo("NavStatusMonitor %s: goal=%s status=%s raw=%s final=%s",
                      self.robot, self.goal_topic, self.status_topic,
                      self.raw_cmd_topic, self.final_cmd_topic)

    def goal_cb(self, msg):
        self.last_goal = msg
        self.last_goal_time = rospy.Time.now()

    def status_cb(self, msg):
        if not msg.status_list:
            self.status_text = "EMPTY"
            self.status_stamp = rospy.Time.now()
            return
        status = msg.status_list[-1]
        self.status_text = STATUS_NAMES.get(status.status, str(status.status))
        self.status_stamp = rospy.Time.now()

    def raw_cmd_cb(self, msg):
        self.last_raw_cmd = msg
        self.last_raw_cmd_time = rospy.Time.now()

    def final_cmd_cb(self, msg):
        self.last_final_cmd = msg
        self.last_final_cmd_time = rospy.Time.now()

    def timer_cb(self, _event):
        now = rospy.Time.now()
        goal_age = -1.0
        gx = 0.0
        gy = 0.0
        if self.last_goal is not None:
            goal_age = (now - self.last_goal_time).to_sec()
            gx = self.last_goal.pose.position.x
            gy = self.last_goal.pose.position.y
        status_age = -1.0
        raw_age = -1.0
        final_age = -1.0
        if self.status_stamp != rospy.Time(0):
            status_age = (now - self.status_stamp).to_sec()
        if self.last_raw_cmd_time != rospy.Time(0):
            raw_age = (now - self.last_raw_cmd_time).to_sec()
        if self.last_final_cmd_time != rospy.Time(0):
            final_age = (now - self.last_final_cmd_time).to_sec()

        rospy.loginfo(
            "[NAV_MON %s] goal=(%.2f,%.2f age=%.2f) status=%s age=%.2f "
            "raw(vx=%.2f,vy=%.2f,wz=%.2f age=%.2f) "
            "final(vx=%.2f,vy=%.2f,wz=%.2f age=%.2f)",
            self.robot, gx, gy, goal_age, self.status_text, status_age,
            self.last_raw_cmd.linear.x, self.last_raw_cmd.linear.y,
            self.last_raw_cmd.angular.z, raw_age,
            self.last_final_cmd.linear.x, self.last_final_cmd.linear.y,
            self.last_final_cmd.angular.z, final_age)

        if self.status_text in ("ACTIVE", "PENDING") and raw_age > 1.5:
            rospy.logwarn_throttle(
                2.0, "[NAV_MON %s] move_base has an active/pending goal but no recent raw cmd",
                self.robot)
        if raw_age >= 0.0 and raw_age < 1.5 and final_age > 1.5:
            rospy.logwarn_throttle(
                2.0, "[NAV_MON %s] raw cmd is fresh but final cmd is stale; check safety filter",
                self.robot)


if __name__ == "__main__":
    rospy.init_node("nav_status_monitor")
    NavStatusMonitor()
    rospy.spin()
