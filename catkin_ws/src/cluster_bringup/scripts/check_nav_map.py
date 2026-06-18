#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_value(grid, x, y):
    origin = grid.info.origin
    yaw = yaw_from_quaternion(origin.orientation)
    dx = x - origin.position.x
    dy = y - origin.position.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    col = int(math.floor(local_x / grid.info.resolution))
    row = int(math.floor(local_y / grid.info.resolution))
    if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
        return None, col, row
    return grid.data[row * grid.info.width + col], col, row


def value_label(value):
    if value is None:
        return "OUTSIDE"
    if value < 0:
        return "UNKNOWN"
    if value >= 100:
        return "LETHAL"
    if value >= 65:
        return "OCCUPIED"
    if value > 0:
        return "INFLATED"
    return "FREE"


def print_point(label, grid_name, grid, x, y):
    value, col, row = grid_value(grid, x, y)
    print("%-7s %-14s x=% .3f y=% .3f cell=(%d,%d) value=%s %s" %
          (label, grid_name, x, y, col, row, str(value), value_label(value)))
    return value


def first_blocked_on_line(grid, sx, sy, gx, gy):
    distance = math.hypot(gx - sx, gy - sy)
    steps = max(1, int(math.ceil(distance / max(grid.info.resolution, 0.01))))
    for index in range(steps + 1):
        ratio = float(index) / float(steps)
        x = sx + ratio * (gx - sx)
        y = sy + ratio * (gy - sy)
        value, _, _ = grid_value(grid, x, y)
        if value is None or value >= 65:
            return x, y, value, ratio
    return None


def cell_world(grid, col, row):
    origin = grid.info.origin
    yaw = yaw_from_quaternion(origin.orientation)
    local_x = (col + 0.5) * grid.info.resolution
    local_y = (row + 0.5) * grid.info.resolution
    x = origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
    y = origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
    return x, y


def nearest_high_cost_cells(grid, robot_x, robot_y, robot_yaw, limit=8):
    cells = []
    for row in range(grid.info.height):
        offset = row * grid.info.width
        for col in range(grid.info.width):
            value = grid.data[offset + col]
            if value < 99:
                continue
            x, y = cell_world(grid, col, row)
            dx = x - robot_x
            dy = y - robot_y
            distance = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) - robot_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            cells.append((distance, bearing, value, x, y))
    cells.sort(key=lambda item: item[0])
    return cells[:limit]


def pose_stamped(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


def main():
    if len(sys.argv) != 3:
        print("Usage: check_nav_map.py GOAL_X GOAL_Y")
        return 2

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    rospy.init_node("check_nav_map", anonymous=True)

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        transform = tf_buffer.lookup_transform(
            "map", "robot1/base_link", rospy.Time(0), rospy.Duration(5.0))
    except Exception as exc:
        print("FAIL: map -> robot1/base_link unavailable: %s" % exc)
        return 1

    start_x = transform.transform.translation.x
    start_y = transform.transform.translation.y
    start_yaw = yaw_from_quaternion(transform.transform.rotation)

    try:
        static_map = rospy.wait_for_message("/map", OccupancyGrid, timeout=8.0)
        global_costmap = rospy.wait_for_message(
            "/robot1/move_base/global_costmap/costmap",
            OccupancyGrid, timeout=8.0)
        local_costmap = rospy.wait_for_message(
            "/robot1/move_base/local_costmap/costmap",
            OccupancyGrid, timeout=8.0)
    except rospy.ROSException as exc:
        print("FAIL: map topic unavailable: %s" % exc)
        return 1

    print("start=(%.3f, %.3f, yaw=%.3f) goal=(%.3f, %.3f)" %
          (start_x, start_y, start_yaw, goal_x, goal_y))
    print("static map: %dx%d resolution=%.3f" %
          (static_map.info.width, static_map.info.height,
           static_map.info.resolution))
    print_point("START", "static_map", static_map, start_x, start_y)
    print_point("GOAL", "static_map", static_map, goal_x, goal_y)
    print_point("START", "global_costmap", global_costmap, start_x, start_y)
    print_point("GOAL", "global_costmap", global_costmap, goal_x, goal_y)
    print_point("START", "local_costmap", local_costmap, start_x, start_y)
    print_point("GOAL", "local_costmap", local_costmap, goal_x, goal_y)

    nearest = nearest_high_cost_cells(
        local_costmap, start_x, start_y, start_yaw)
    if nearest:
        print("nearest local cells with cost >= 99:")
        for distance, bearing, value, x, y in nearest:
            print("  distance=%.3fm bearing=% .1fdeg value=%d world=(%.3f,%.3f)" %
                  (distance, math.degrees(bearing), value, x, y))
    else:
        print("nearest local cells: none with cost >= 99")

    blocked = first_blocked_on_line(global_costmap,
                                    start_x, start_y, goal_x, goal_y)
    if blocked is None:
        print("direct line: no occupied cell (global costmap)")
    else:
        print("direct line: blocked at x=%.3f y=%.3f value=%s progress=%.0f%%" %
              (blocked[0], blocked[1], str(blocked[2]), blocked[3] * 100.0))

    local_blocked = first_blocked_on_line(local_costmap,
                                          start_x, start_y, goal_x, goal_y)
    if local_blocked is None:
        print("local direct line: no occupied cell")
    else:
        print("local direct line: blocked at x=%.3f y=%.3f value=%s progress=%.0f%%" %
              (local_blocked[0], local_blocked[1], str(local_blocked[2]),
               local_blocked[3] * 100.0))

    service_name = "/robot1/move_base/make_plan"
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        request = GetPlanRequest()
        request.start = pose_stamped(start_x, start_y, start_yaw)
        request.goal = pose_stamped(goal_x, goal_y, start_yaw)
        request.tolerance = 0.20
        response = rospy.ServiceProxy(service_name, GetPlan)(request)
        print("make_plan: %d poses" % len(response.plan.poses))
        if not response.plan.poses:
            print("RESULT: FAIL - global planner cannot connect start and goal")
            return 1
    except Exception as exc:
        print("FAIL: make_plan service error: %s" % exc)
        return 1

    print("RESULT: PASS - global path exists; inspect local costmap/DWA next")
    return 0


if __name__ == "__main__":
    sys.exit(main())
