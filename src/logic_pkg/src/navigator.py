#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

class NavigatorNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('navigator_node', anonymous=True)

        # Subscribe to the topic with the offset and angle data
        self.data_sub = rospy.Subscriber("line_offset_angle", Float32MultiArray, self.callback)

    def callback(self, data):
        # Extract the offset and angle data
        offset_mm = data.data[0]
        angle_deg = data.data[1]

        # Print the offset and angle to the console
        rospy.loginfo(f"Received Data - Offset: {offset_mm:.2f} mm, Angle: {angle_deg:.2f} degrees")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    navigator_node = NavigatorNode()
    navigator_node.run()
