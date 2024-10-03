#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String


# define robot status
robot_status_dict = {
    'STRAIGHT': '10',
    'TURN_RIGHT': '11',
    'TURN_LEFT': '12',
    'PAUSE': '13',
    'STOP': '14'
}

rotation_status_dict = {
    'LEFT': '0',
    'CENTER': '1',
    'RIGHT': '2'
}

clump_status_dict = {
    'CLUMP_OFF': '0',
    'CLUMP_ON': '1'
}

rail_status_dict = {
    'RAIL_OFF': '0',
    'RAIL_ON': '1'
}

panel_status_dict = {
    'PANEL_ALL_ON': '0',
    'PANEL_RIGHT_FORWARD_OFF': '1',
    'PANEL_RIGHT_BACKWARD_OFF': '2',
    'PANEL_LEFT_FORWARD_OFF': '3',
    'PANEL_LEFT_BACKWARD_OFF': '4'
}

wheel_status_dict = {
    'WHEEL_STOP': '0',
    'WHEEL_STRAIGHT': '1',
    'WHEEL_RIGHT': '2',
    'WHEEL_LEFT': '3'
}


class NavigatorNode:
    def __init__(self):
        rospy.init_node('navigator_node', anonymous=True)

        # Subscribe to the topic with the offset and angle data
        self.data_sub = rospy.Subscriber("image_info", Float32MultiArray, self.callback)

        # Publisher for motor speed commands
        self.rotation_position_pub = rospy.Publisher("rotation_position", String, queue_size=10)
        self.clump_position_pub = rospy.Publisher("clump_position", String, queue_size=10)
        self.rail_position_pub = rospy.Publisher("rail_position", String, queue_size=10)
        self.panel_position_pub = rospy.Publisher("panel_position", String, queue_size=10)
        self.wheel_speed_pub = rospy.Publisher("wheel_speeds", Int32MultiArray, queue_size=10)

        self.robot_status = robot_status_dict['STRAIGHT']

        self.rotatoin_msg = String()
        self.clump_msg = String()
        self.rail_msg = String()
        self.panel_msg = String()
        self.wheel_speeds_msg = Int32MultiArray()
        self.left_motor_speed = 100
        self.right_motor_speed = 100

        # initialize publish data
        self.rotatoin_msg.data = rotation_status_dict['CENTER']
        self.clump_msg.data = clump_status_dict['CLUMP_OFF']
        self.rail_msg.data = rail_status_dict['RAIL_ON']
        self.panel_msg.data = panel_status_dict['PANEL_ALL_ON']
        self.wheel_speeds_msg = [self.left_motor_speed, self.right_motor_speed]

        self.acceptable_offset_mm = 0.5
        self.acceptable_angle_deg = 0.5
        self.acceptable_arc_distance_mm = 5
        self.log_counter = 0

    
    def publish_msg(self):
        self.rotation_position_pub.publish(self.rotatoin_msg)
        self.clump_position_pub.publish(self.clump_msg)
        self.rail_position_pub.publish(self.panel_msg)
        self.panel_position_pub.publish(self.panel_msg)
        self.wheel_speed_pub.publish(self.wheel_speeds_msg)
    
    def cmd_check(self, cmd):
        check_result = True

        if not cmd.startswith('S'):
            self.error_msg = 'command must start with "S"'
            check_result = False
        if not cmd.endswith('E'):
            self.error_msg = 'command must end with "E"'
            check_result = False
        if len(cmd) != 7:
            self.error_msg = 'The command must be entered in the following format S00000E'
            check_result = False
        if not cmd[1] in rotation_status_dict.values():
            self.error_msg = '1st number must be 0~2'
            check_result = False
        elif not cmd[2] in clump_status_dict.values():
            self.error_msg = '2nd number must be 0 or 1'
            check_result = False
        elif not cmd[3] in rail_status_dict.values():
            self.error_msg = '3rd number must be 0 or 1'
            check_result = False
        elif not cmd[4] in panel_status_dict.values():
            self.error_msg = '4th number must be 0~4'
            check_result = False
        elif not cmd[5] in wheel_status_dict.values():
            self.error_msg = '5th number must be 0~3'
            check_result = False

        return check_result
    
    def motor_speed_set(self, wheel_cmd):
        if wheel_cmd == wheel_status_dict['WHEEL_STOP']:    
            self.left_motor_speed = 0
            self.right_motor_speed = 0
        elif wheel_cmd == wheel_status_dict['WHEEL_STRAIGHT']:
            self.left_motor_speed = 100
            self.right_motor_speed = 100
        elif wheel_cmd == wheel_status_dict['WHEEL_LEFT']:
            self.left_motor_speed = -50
            self.right_motor_speed = 50
        elif wheel_cmd == wheel_status_dict['WHEEL_RIGHT']:
            self.left_motor_speed = 50
            self.right_motor_speed = -50
    
    def robot_status_set(self, wheel_cmd):
        if wheel_cmd == wheel_status_dict['WHEEL_STOP']:
            self.robot_status = robot_status_dict['PAUSE']
        elif wheel_cmd == wheel_status_dict['WHEEL_STRAIGHT']:
            self.robot_status = robot_status_dict['STRAIGHT']
        elif wheel_cmd == wheel_status_dict['WHEEL_LEFT']:
            self.robot_status = robot_status_dict['TURN_LEFT']
        elif wheel_cmd == wheel_status_dict['WHEEL_RIGHT']:
            self.robot_status = robot_status_dict['TURN_RIGHT']

        

    def callback(self, data):
 
        # Extract the offset and angle data
        offset_mm = data.data[0]
        angle_deg = data.data[1]
        arc_distance_mm = data.data[2]
        stop_signal = int(data.data[3])

        if self.robot_status == robot_status_dict['STRAIGHT']:
            if abs(offset_mm) > self.acceptable_offset:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])
            elif stop_signal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])

        elif self.robot_status == robot_status_dict['TURN_RIGHT']:
            if abs(arc_distance_mm) > self.acceptable_arc_distance_mm:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])
            elif stop_signal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])

        elif self.robot_status == robot_status_dict['TURN_LEFT']:
            if abs(arc_distance_mm) > self.acceptable_arc_distance_mm:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])
            elif stop_signal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'])

        elif self.robot_status == robot_status_dict['PAUSE']:
            cmd = input('Please input command...')
            while self.cmd_check(cmd):
                cmd = input(f'command error: {self.error_msg}')

            rotation_cmd = cmd[1]
            clump_cmd = cmd[2]
            rail_cmd = cmd[3]
            panel_cmd = cmd[4]
            wheel_cmd = cmd[5]

            self.rotatoin_msg.data = rotation_cmd
            self.clump_msg.data = clump_cmd
            self.rail_msg.data = rail_cmd
            self.panel_msg.data = panel_cmd
            self.motor_speed_set(wheel_cmd)
            self.robot_status_set(wheel_cmd)

        elif self.robot_status == robot_status_dict['STOP']:
            pass
        

        # Print the offset and angle to the console
        self.log_counter += 1
        if self.log_counter >= 20:
            rospy.loginfo(f"Received deviation - offset: {offset_mm}, angle: {angle_deg}, motor speed: {right_motor_speed}")
            self.log_counter = 0

        # Publish tmessages
        self.publish_msg()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    navigator_node = NavigatorNode()
    navigator_node.run()
