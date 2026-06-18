#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class AssignedGoalRelay(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_goal_topic", "/robot2/assigned_goal")
        self.output_topic = rospy.get_param("~output_goal_topic", "/robot2/move_base_simple/goal")
        self.required_frame = rospy.get_param("~required_frame", "map")
        self.min_xy_delta = rospy.get_param("~min_xy_delta", 0.18)
        self.min_yaw_delta = rospy.get_param("~min_yaw_delta", 0.25)
        self.min_publish_period = rospy.Duration(
            rospy.get_param("~min_publish_period", 0.8))
        self.force_republish_period = rospy.Duration(
            rospy.get_param("~force_republish_period", 3.0))
        self.debug_enabled = rospy.get_param("~debug_enabled", False)
        self.debug_period = rospy.get_param("~debug_period", 1.0)

        self.last_goal = None
        self.last_input_goal = None
        self.last_publish_time = rospy.Time(0)
        self.received_count = 0
        self.published_count = 0
        self.drop_reason = "waiting"
        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PoseStamped, self.callback,
                                    queue_size=10)
        if self.debug_enabled:
            self.timer = rospy.Timer(rospy.Duration(self.debug_period),
                                     self.debug_timer)
        rospy.loginfo("AssignedGoalRelay: %s -> %s", self.input_topic, self.output_topic)

    @staticmethod
    def yaw_from_pose(msg):
        q = msg.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    @staticmethod
    def finite(value):
        return not (math.isnan(value) or math.isinf(value))

    def valid_goal(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(self.finite(v) for v in values):
            self.drop_reason = "non_finite"
            rospy.logwarn_throttle(2.0, "Ignoring non-finite assigned goal")
            return False
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 0.5 or norm > 1.5:
            self.drop_reason = "bad_quaternion"
            rospy.logwarn_throttle(2.0, "Ignoring assigned goal with bad quaternion norm %.3f",
                                   norm)
            return False
        return True

    def should_publish(self, msg):
        now = rospy.Time.now()
        if not self.valid_goal(msg):
            return False
        if self.required_frame and msg.header.frame_id != self.required_frame:
            self.drop_reason = "frame"
            rospy.logwarn_throttle(2.0, "Ignoring goal in frame %s, expected %s",
                                   msg.header.frame_id, self.required_frame)
            return False
        if self.last_goal is None:
            return True
        if now - self.last_publish_time < self.min_publish_period:
            self.drop_reason = "min_period"
            return False
        if now - self.last_publish_time > self.force_republish_period:
            return True

        dx = msg.pose.position.x - self.last_goal.pose.position.x
        dy = msg.pose.position.y - self.last_goal.pose.position.y
        if math.hypot(dx, dy) >= self.min_xy_delta:
            return True

        yaw = self.yaw_from_pose(msg)
        last_yaw = self.yaw_from_pose(self.last_goal)
        if abs(self.angle_diff(yaw, last_yaw)) >= self.min_yaw_delta:
            return True
        self.drop_reason = "below_delta"
        return False

    def callback(self, msg):
        self.received_count += 1
        self.last_input_goal = msg
        if not self.should_publish(msg):
            return
        out = PoseStamped()
        out.header = msg.header
        out.header.stamp = rospy.Time.now()
        out.pose = msg.pose
        self.pub.publish(out)
        self.last_goal = out
        self.last_publish_time = out.header.stamp
        self.published_count += 1
        self.drop_reason = "published"

    def debug_timer(self, _event):
        now = rospy.Time.now()
        goal_age = -1.0
        last_x = 0.0
        last_y = 0.0
        if self.last_goal is not None:
            goal_age = (now - self.last_publish_time).to_sec()
            last_x = self.last_goal.pose.position.x
            last_y = self.last_goal.pose.position.y
        rospy.loginfo("[GOAL_RELAY] %s -> %s recv=%d pub=%d last=(%.2f,%.2f) age=%.2f reason=%s",
                      self.input_topic, self.output_topic,
                      self.received_count, self.published_count,
                      last_x, last_y, goal_age, self.drop_reason)


if __name__ == "__main__":
    rospy.init_node("assigned_goal_relay")
    AssignedGoalRelay()
    rospy.spin()
