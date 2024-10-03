#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray

class NavigatorNode:
    def __init__(self):
        self.acceptable_offset = 0.5
        self.acceptable_angle = 0.5
        self.log_counter = 0
        # Initialize the node
        rospy.init_node('navigator_node', anonymous=True)

        # Subscribe to the topic with the offset and angle data
        self.data_sub = rospy.Subscriber("line_offset_angle", Float32MultiArray, self.callback)

        # Publisher for motor speed commands
        self.motor_speed_pub = rospy.Publisher("motor_speeds", Int32MultiArray, queue_size=10)

    def callback(self, data):
 
        # Extract the offset and angle data
        offset_mm = data.data[0]
        angle_deg = data.data[1]

        # Print the offset and angle to the console
        # Calculate the motor speeds (this is a placeholder for your calculations)
        left_motor_speed = 0  # Replace with actual calculation
        right_motor_speed = 0  # Replace with actual calculation

        if abs(offset_mm) < self.acceptable_offset and abs(angle_deg) < self.acceptable_angle:
            left_motor_speed = 100
            right_motor_speed = 100
        else:
            left_motor_speed = 0
            right_motor_speed = 0
        
        self.log_counter += 1
        if self.log_counter >= 20:
            rospy.loginfo(f"Received deviation - offset: {offset_mm}, angle: {angle_deg}, motor speed: {right_motor_speed}")
            self.log_counter = 0

        # Prepare the data to publish
        motor_speeds_msg = Int32MultiArray()
        motor_speeds_msg.data = [left_motor_speed, right_motor_speed]

        # Publish the motor speed commands
        self.motor_speed_pub.publish(motor_speeds_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    navigator_node = NavigatorNode()
    navigator_node.run()
