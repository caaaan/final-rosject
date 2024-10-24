#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import curses
import sys
import signal


class TeleopRobot():
    def __init__(self, topic="/locobot/mobile_base/commands/velocity"):
        self.topic = topic
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Initialize ROS node and publisher
        rospy.init_node('teleop')
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Initialize curses screen
        self.screen = curses.initscr()
        curses.cbreak()
        self.screen.keypad(True)
        curses.noecho()
        self.screen.clear()

        # Print instructions
        self.screen.addstr(0, 0, "Use arrow keys to teleoperate the robot:")
        self.screen.addstr(2, 0, "    up    : increase linear speed")
        self.screen.addstr(3, 0, "    down  : decrease linear speed")
        self.screen.addstr(4, 0, "    left  : increase angular speed")
        self.screen.addstr(5, 0, "    right : decrease angular speed")
        self.screen.addstr(7, 0, "Current speeds:")
        self.screen.refresh()

    def run(self):
        # Set signal handler for SIGINT (CTRL+C)
        signal.signal(signal.SIGINT, self.cleanup)

        while not rospy.is_shutdown():
            # Get input from the user
            key = self.screen.getch()
            if key == curses.KEY_UP:
                self.linear_speed = min(self.linear_speed + 0.1, 1.0)
            elif key == curses.KEY_DOWN:
                self.linear_speed = max(self.linear_speed - 0.1, -1.0)
            elif key == curses.KEY_LEFT:
                self.angular_speed = min(self.angular_speed + 0.1, 1.0)
            elif key == curses.KEY_RIGHT:
                self.angular_speed = max(self.angular_speed - 0.1, -1.0)

            # Create Twist message
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

            # Publish Twist message
            self.pub.publish(twist)

            # Display current speeds
            self.screen.addstr(8, 0, "    linear : {:.1f}".format(self.linear_speed))
            self.screen.addstr(9, 0, "    angular: {:.1f}".format(self.angular_speed))
            self.screen.refresh()

            self.rate.sleep()

    def cleanup(self, signal, frame):
        # Stop the robot when the program quits
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

        # Restore terminal settings
        curses.nocbreak()
        self.screen.keypad(False)
        curses.echo()
        curses.endwin()

        sys.exit(0)


if __name__ == '__main__':
    topic = "/locobot/mobile_base/commands/velocity"
    if len(sys.argv) > 1:
        topic = sys.argv[1]

    try:
        teleop = TeleopRobot(topic)
        teleop.run()
    except rospy.ROSInterruptException:
        pass
