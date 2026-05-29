#!/usr/bin/env python
"""
Keyboard teleoperation for LIMO S2 cluster control.
Publishes velocity commands and switches system modes.

Keys:
  w/s    : forward / backward
  a/d    : turn left / right
  q      : stop / quit

  z      : IDLE mode
  x      : TELEOP mode
  c      : FORMATION mode (COLUMN)
  v      : FOLLOW mode
  1      : COLUMN formation
  2      : LINE formation
  3      : TRIANGLE_LEFT formation
  4      : TRIANGLE_RIGHT formation
  5      : switch follower control mode
  6      : toggle avoidance filter
  0      : return car1 to startup home pose

CTRL-C to quit.
"""

import rospy
import sys
import select
import termios
import tty

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String
from cluster_msgs.srv import SetMode, SetFormation


class TeleopKeyboard:
    def __init__(self):
        self.ns = rospy.get_namespace().strip('/')
        rospy.loginfo("TeleopKeyboard starting in namespace: %s", self.ns)

        # Publisher
        teleop_topic = '/' + self.ns + '/teleop_vel' if self.ns else '/teleop_vel'
        self.pub = rospy.Publisher(teleop_topic, Twist, queue_size=10)
        self.control_mode_pub = rospy.Publisher(
            '/robot2/follower_control_mode', String, queue_size=1, latch=True)
        self.avoidance_pub = rospy.Publisher(
            '/robot2/avoidance_enabled', Bool, queue_size=1, latch=True)
        self.return_home_pub = rospy.Publisher(
            '/robot1/return_home', Bool, queue_size=1)

        # Service clients
        rospy.wait_for_service('/robot1/set_mode', timeout=10.0)
        rospy.wait_for_service('/robot1/set_formation', timeout=10.0)
        self.set_mode = rospy.ServiceProxy('/robot1/set_mode', SetMode)
        self.set_formation = rospy.ServiceProxy('/robot1/set_formation', SetFormation)

        # Speed settings
        self.speed = rospy.get_param('~speed', 0.3)
        self.turn = rospy.get_param('~turn', 0.5)
        self.control_modes = ['body_orbit', 'wheeltec_global']
        self.control_mode_index = 0
        self.avoidance_enabled = True

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.running = True
        self.vx = 0.0
        self.vz = 0.0

        self.print_help()
        rospy.loginfo("TeleopKeyboard ready. Use w/a/s/d to drive, z/x/c/v to switch modes.")

    def print_help(self):
        print("\n" + "=" * 50)
        print("  LIMO S2 Cluster Control - Keyboard Teleop")
        print("=" * 50)
        print("  Driving:")
        print("    w/s  : forward / backward")
        print("    a/d  : turn left / right")
        print("  Modes:")
        print("    z    : IDLE")
        print("    x    : TELEOP")
        print("    c    : FORMATION (COLUMN)")
        print("    v    : FOLLOW")
        print("  Formations:")
        print("    1    : COLUMN")
        print("    2    : LINE")
        print("    3    : TRIANGLE_LEFT")
        print("    4    : TRIANGLE_RIGHT")
        print("    5    : switch follower mode (body_orbit / wheeltec_global)")
        print("    6    : toggle avoidance filter")
        print("    0    : return car1 to startup home pose")
        print("  q / CTRL-C : quit")
        print("=" * 50 + "\n")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def switch_mode(self, mode, formation=0):
        try:
            resp = self.set_mode(
                mode=mode,
                formation=formation,
                offset_x=0.0,
                offset_y=0.0,
                offset_yaw=0.0
            )
            if resp.success:
                mode_names = {0: "IDLE", 1: "TELEOP", 2: "FORMATION", 3: "FOLLOW"}
                rospy.loginfo("Mode: %s", mode_names.get(mode, "UNKNOWN"))
            else:
                rospy.logwarn("SetMode failed: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def switch_formation(self, formation):
        try:
            resp = self.set_formation(formation=formation)
            if resp.success:
                names = {0: "COLUMN", 1: "LINE", 2: "TRIANGLE_LEFT", 3: "TRIANGLE_RIGHT"}
                rospy.loginfo("Formation: %s", names.get(formation, "UNKNOWN"))
            else:
                rospy.logwarn("SetFormation failed: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def switch_follower_control_mode(self):
        self.control_mode_index = (self.control_mode_index + 1) % len(self.control_modes)
        mode = self.control_modes[self.control_mode_index]
        self.control_mode_pub.publish(String(data=mode))
        rospy.loginfo("Follower control mode: %s", mode)

    def return_home(self):
        self.return_home_pub.publish(Bool(data=True))
        rospy.loginfo("Return car1 to startup home pose")

    def toggle_avoidance(self):
        self.avoidance_enabled = not self.avoidance_enabled
        self.avoidance_pub.publish(Bool(data=self.avoidance_enabled))
        rospy.loginfo("Avoidance filter: %s",
                      "enabled" if self.avoidance_enabled else "disabled")

    def run(self):
        rate = rospy.Rate(20)
        while self.running and not rospy.is_shutdown():
            key = self.get_key()

            # Velocity keys
            if key == 'w':
                self.vx = self.speed
                self.vz = 0.0
            elif key == 's':
                self.vx = -self.speed
                self.vz = 0.0
            elif key == 'a':
                self.vz = self.turn
                self.vx = 0.0
            elif key == 'd':
                self.vz = -self.turn
                self.vx = 0.0
            elif key == ' ':
                self.vx = 0.0
                self.vz = 0.0
            # Mode keys
            elif key == 'z':
                self.switch_mode(0)
            elif key == 'x':
                self.switch_mode(1)
            elif key == 'c':
                self.switch_mode(2, 0)
            elif key == 'v':
                self.switch_mode(3)
            # Formation keys
            elif key == '1':
                self.switch_formation(0)
            elif key == '2':
                self.switch_formation(1)
            elif key == '3':
                self.switch_formation(2)
            elif key == '4':
                self.switch_formation(3)
            elif key == '5':
                self.switch_follower_control_mode()
            elif key == '6':
                self.toggle_avoidance()
            elif key == '0':
                self.return_home()
            elif key == 'q':
                self.running = False
                break
            elif key == '\x03':
                self.running = False
                break
            else:
                # No recognized key - stop moving (auto-release)
                self.vx = 0.0
                self.vz = 0.0

            # Publish velocity
            twist = Twist()
            twist.linear.x = self.vx
            twist.angular.z = self.vz
            self.pub.publish(twist)

            rate.sleep()

        # Stop on exit
        twist = Twist()
        self.pub.publish(twist)
        rospy.loginfo("TeleopKeyboard stopped.")

    def shutdown(self):
        self.running = False


if __name__ == '__main__':
    rospy.init_node('teleop_keyboard', disable_signals=True)
    teleop = TeleopKeyboard()
    rospy.on_shutdown(teleop.shutdown)

    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
                          termios.tcgetattr(sys.stdin))
