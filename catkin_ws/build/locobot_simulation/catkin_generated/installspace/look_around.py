#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class PublisherNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('publisher_node', anonymous=True)

        # Create publishers
        self.pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
        self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

        # Set the publishing rate
        self.rate = rospy.Rate(100) # 1 Hz

        # Set initial commmands for pan & tilt
        self.pan_cmd = 0
        self.tilt_cmd = 0.3
        self.delta = 0.01

    def publish_topics(self):
        while not rospy.is_shutdown():
            # Create messages
            topic1_msg = self.pan_cmd
            topic2_msg = self.tilt_cmd

            # Publish messages
            rospy.loginfo('Publishing command: {}'.format(self.pan_cmd))
            self.pan_publisher.publish(topic1_msg)
            self.tilt_publisher.publish(topic2_msg)

            if self.pan_cmd >= 1.8 or self.pan_cmd <= -1.8: self.delta *= -1

            self.pan_cmd += self.delta
            #self.tilt_cmd += 0.1

            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node = PublisherNode()
        publisher_node.publish_topics()
    except rospy.ROSInterruptException:
        pass
