#!/usr/bin/env python
import numpy as np

import rospy
import skfuzzy as fuzz
from skfuzzy import control as ctrl
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
    'ALL_OFF': '0',
    'FORWARD_ON': '1',
    'BACKWARD_ON': '2',
    'ALL_ON':'3'
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


# ファジィ変数の定義
offset = ctrl.Antecedent(np.arange(-50, 51, 1), 'offset')  # 中心からの距離のずれ
angle = ctrl.Antecedent(np.arange(-45, 46, 1), 'angle')  # y方向の角度のずれ
motor_speed = ctrl.Consequent(np.arange(-100, 101, 1), 'motor_speed')  # 左右モーターの速度調整量

# ファジィセットの定義（メンバーシップ関数）
offset['negative'] = fuzz.trimf(offset.universe, [-50, -50, 0])
offset['zero'] = fuzz.trimf(offset.universe, [-50, 0, 50])
offset['positive'] = fuzz.trimf(offset.universe, [0, 50, 50])

angle['negative'] = fuzz.trimf(angle.universe, [-45, -45, 0])
angle['zero'] = fuzz.trimf(angle.universe, [-45, 0, 45])
angle['positive'] = fuzz.trimf(angle.universe, [0, 45, 45])

motor_speed['negative'] = fuzz.trimf(motor_speed.universe, [-100, -100, 0])
motor_speed['zero'] = fuzz.trimf(motor_speed.universe, [-100, 0, 100])
motor_speed['positive'] = fuzz.trimf(motor_speed.universe, [0, 100, 100])

# ファジィルールの定義
rule1 = ctrl.Rule(offset['negative'] & angle['negative'], motor_speed['positive'])
rule2 = ctrl.Rule(offset['negative'] & angle['zero'], motor_speed['positive'])
rule3 = ctrl.Rule(offset['negative'] & angle['positive'], motor_speed['negative'])
rule4 = ctrl.Rule(offset['zero'] & angle['negative'], motor_speed['positive'])
rule5 = ctrl.Rule(offset['zero'] & angle['zero'], motor_speed['zero'])
rule6 = ctrl.Rule(offset['zero'] & angle['positive'], motor_speed['negative'])
rule7 = ctrl.Rule(offset['positive'] & angle['negative'], motor_speed['positive'])
rule8 = ctrl.Rule(offset['positive'] & angle['zero'], motor_speed['negative'])
rule9 = ctrl.Rule(offset['positive'] & angle['positive'], motor_speed['negative'])

# 制御システムの構築
motor_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
motor_simulation = ctrl.ControlSystemSimulation(motor_ctrl)


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
    
    def motor_speed_set(self, wheel_cmd, speed_weight):
        if wheel_cmd == wheel_status_dict['WHEEL_STOP']:    
            self.left_motor_speed = 0
            self.right_motor_speed = 0
        elif wheel_cmd == wheel_status_dict['WHEEL_STRAIGHT']:
            if speed_weight == 0:
                self.left_motor_speed = 100
                self.right_motor_speed = 100
            elif speed_weight > 0:
                self.left_motor_speed = 100
                self.right_motor_speed = 100 * (1 + abs(speed_weight))
            elif speed_weight < 0:
                self.left_motor_speed = 100 * (1 + abs(speed_weight))
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
    
    def motor_speed_fuzzy(self, offset_mm, angle):
        motor_simulation.input['offset'] = offset_mm
        motor_simulation.input['angle'] = angle

        # 推論を実行
        motor_simulation.compute()

        # 出力を得る
        speed = motor_simulation.output['motor_speed']
        weight = speed / 100
        return weight
        

    def callback(self, data):
 
        # Extract the offset and angle data
        offset_mm = data.data[0]
        angle_deg = data.data[1]
        arc_distance_mm = data.data[2]
        pause_segnal = int(data.data[3])
        motor_speed_weight = self.motor_speed_fuzzy(offset_mm, angle_deg)

        if self.robot_status == robot_status_dict['STRAIGHT']:
            if abs(offset_mm) <= self.acceptable_offset_mm and abs(angle_deg) <= self.acceptable_angle_deg:
                self.motor_speed_set(wheel_status_dict['WHEEL_STRAIGHT'], motor_speed_weight)
            elif abs(offset_mm) > self.acceptable_offset:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)
            elif pause_segnal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)

        elif self.robot_status == robot_status_dict['TURN_RIGHT']:
            if abs(arc_distance_mm) > self.acceptable_arc_distance_mm:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)
            elif pause_segnal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)

        elif self.robot_status == robot_status_dict['TURN_LEFT']:
            if abs(arc_distance_mm) > self.acceptable_arc_distance_mm:
                self.robot_status = robot_status_dict['STOP']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)
            elif pause_segnal:
                self.robot_status = robot_status_dict['PAUSE']
                self.motor_speed_set(wheel_status_dict['WHEEL_STOP'], motor_speed_weight)

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
