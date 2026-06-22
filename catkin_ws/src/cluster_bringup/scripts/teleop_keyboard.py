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
  3      : CIRCLE_SHOW formation
  4      : TRIANGLE formation
  e      : safely exit CIRCLE_SHOW and restore the prior formation
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
        self.pub = rospy.Publisher(teleop_topic, Twist, queue_size=1)
        self.control_mode_pub = rospy.Publisher(
            '/robot2/follower_control_mode', String, queue_size=1, latch=True)
        self.avoidance_pub = rospy.Publisher(
            '/robot2/avoidance_enabled', Bool, queue_size=1, latch=True)
        self.return_home_pub = rospy.Publisher(
            '/robot1/return_home', Bool, queue_size=1)
        self.circle_exit_pub = rospy.Publisher(
            '/robot1/circle_exit', Bool, queue_size=1)

        # Service clients
        rospy.wait_for_service('/robot1/set_mode', timeout=10.0)
        rospy.wait_for_service('/robot1/set_formation', timeout=10.0)
        self.set_mode = rospy.ServiceProxy('/robot1/set_mode', SetMode)
        self.set_formation = rospy.ServiceProxy('/robot1/set_formation', SetFormation)

        # Speed settings
        self.speed = rospy.get_param('~speed', 0.3)
        self.turn = rospy.get_param('~turn', 0.5)
        self.key_timeout = rospy.get_param('~key_timeout', 0.01)
        self.key_hold_timeout = rospy.get_param('~key_hold_timeout', 0.22)
        self.key_drain_limit = int(rospy.get_param('~key_drain_limit', 128))
        self.service_debounce = rospy.get_param('~service_debounce', 0.35)
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)
        self.control_modes = ['body_orbit', 'wheeltec_global']
        self.control_mode_index = 0
        self.avoidance_enabled = True

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.running = True
        self.vx = 0.0
        self.vz = 0.0
        self.last_motion_key_time = rospy.Time(0)
        self.last_service_key = ''
        self.last_service_key_time = rospy.Time(0)

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
        print("    3    : CIRCLE_SHOW")
        print("    4    : TRIANGLE")
        print("    e    : safely exit CIRCLE_SHOW and restore prior formation")
        print("    5    : switch follower mode (body_orbit / wheeltec_global)")
        print("    6    : toggle avoidance filter")
        print("    0    : return car1 to startup home pose")
        print("  q / CTRL-C : quit")
        print("=" * 50 + "\n")

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        key = ''
        drained = 0
        while rlist and drained < self.key_drain_limit:
            ch = sys.stdin.read(1)
            if ch == '\x03':
                return ch
            key = ch
            drained += 1
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            # SSH key-repeat can build a large backlog. Drop the remainder so
            # old WASD repeats cannot delay later mode/formation keys.
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        return key

    def service_key_ready(self, key):
        now = rospy.Time.now()
        if (self.last_service_key == key and self.last_service_key_time != rospy.Time(0) and
                (now - self.last_service_key_time).to_sec() < self.service_debounce):
            return False
        self.last_service_key = key
        self.last_service_key_time = now
        return True

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
            resp = self.set_mode(
                mode=2,
                formation=formation,
                offset_x=0.0,
                offset_y=0.0,
                offset_yaw=0.0
            )
            if resp.success:
                names = {0: "COLUMN", 1: "LINE", 2: "CIRCLE_SHOW", 3: "TRIANGLE"}
                rospy.loginfo("Mode: FORMATION, Formation: %s",
                              names.get(formation, "UNKNOWN"))
            else:
                rospy.logwarn("Set formation mode failed: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def switch_follower_control_mode(self):
        self.control_mode_index = (self.control_mode_index + 1) % len(self.control_modes)
        mode = self.control_modes[self.control_mode_index]
        self.control_mode_pub.publish(String(data=mode))
        rospy.loginfo("Follower control mode: %s", mode)

    def return_home(self):
        if self.return_home_pub.get_num_connections() == 0:
            rospy.logwarn("No subscriber on /robot1/return_home yet")
        for _ in range(3):
            self.return_home_pub.publish(Bool(data=True))
            rospy.sleep(0.03)
        rospy.loginfo("Return car1 to startup home pose")

    def toggle_avoidance(self):
        self.avoidance_enabled = not self.avoidance_enabled
        self.avoidance_pub.publish(Bool(data=self.avoidance_enabled))
        rospy.loginfo("Avoidance filter: %s",
                      "enabled" if self.avoidance_enabled else "disabled")

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        tty.setraw(sys.stdin.fileno())
        try:
            while self.running and not rospy.is_shutdown():
                key = self.get_key()
                now = rospy.Time.now()

                # Velocity keys. Keep the last velocity briefly so SSH/key-repeat
                # jitter does not make the base pulse stop between repeated keys.
                if key == 'w':
                    self.vx = self.speed
                    self.vz = 0.0
                    self.last_motion_key_time = now
                elif key == 's':
                    self.vx = -self.speed
                    self.vz = 0.0
                    self.last_motion_key_time = now
                elif key == 'a':
                    self.vz = self.turn
                    self.vx = 0.0
                    self.last_motion_key_time = now
                elif key == 'd':
                    self.vz = -self.turn
                    self.vx = 0.0
                    self.last_motion_key_time = now
                elif key == ' ':
                    self.vx = 0.0
                    self.vz = 0.0
                    self.last_motion_key_time = rospy.Time(0)
                # Mode keys
                elif key == 'z':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Mode: IDLE")
                        self.switch_mode(0)
                    self.vx = 0.0
                    self.vz = 0.0
                    self.last_motion_key_time = rospy.Time(0)
                elif key == 'x':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Mode: TELEOP")
                        self.switch_mode(1)
                elif key == 'c':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Mode: FORMATION")
                        self.switch_mode(2, 0)
                elif key == 'v':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Mode: FOLLOW")
                        self.switch_mode(3)
                # Formation keys
                elif key == '1':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Formation: COLUMN")
                        self.switch_formation(0)
                elif key == '2':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Formation: LINE")
                        self.switch_formation(1)
                elif key == '3':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Formation: CIRCLE_SHOW")
                        self.switch_formation(2)
                elif key == '4':
                    if self.service_key_ready(key):
                        rospy.loginfo("Request Formation: TRIANGLE")
                        self.switch_formation(3)
                elif key == 'e':
                    if self.service_key_ready(key):
                        self.vx = 0.0
                        self.vz = 0.0
                        self.last_motion_key_time = rospy.Time(0)
                        self.circle_exit_pub.publish(Bool(data=True))
                        rospy.loginfo("Request CIRCLE_SHOW safe exit")
                elif key == '5':
                    if self.service_key_ready(key):
                        self.switch_follower_control_mode()
                elif key == '6':
                    if self.service_key_ready(key):
                        self.toggle_avoidance()
                elif key == '0':
                    if self.service_key_ready(key):
                        self.return_home()
                elif key == 'q':
                    self.running = False
                    break
                elif key == '\x03':
                    self.running = False
                    break
                elif key == '':
                    if (self.last_motion_key_time == rospy.Time(0) or
                            (now - self.last_motion_key_time).to_sec() > self.key_hold_timeout):
                        self.vx = 0.0
                        self.vz = 0.0

                # Publish velocity continuously at a stable rate.
                twist = Twist()
                twist.linear.x = self.vx
                twist.angular.z = self.vz
                self.pub.publish(twist)

                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSANOW, self.settings)

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
        termios.tcsetattr(sys.stdin, termios.TCSANOW, teleop.settings)
