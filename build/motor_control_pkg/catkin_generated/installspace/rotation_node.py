#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from motor_control_pkg.msg import ModbusWrite
rotation_status_dict = {
    'LEFT': '0',
    'CENTER': '1',
    'RIGHT': '2'
}

class RotationPositionNode:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('rotation_position_node', anonymous=True)
        self.rotation_status = rotation_status_dict['CENTER']
        self.slave_id = 1
        self.rotation_position_sub = rospy.Subscriber('/rotation_position', String, self.callback, queue_size=1)
        self.modbus_write_pub = rospy.Publisher('/modbus_request', ModbusWrite, queue_size=1)

        # 初期設定
        self.presetting()

    def presetting(self):
        # type(absolute), step(0), speed(500), trigger(step)
        self.send_modbus_command(0x0058, [0, 0, 0, 1, 0, 0, 0, 500], self.slave_id)
        self.send_modbus_command(0x0066, [0xffff, 0xfffb], self.slave_id)

        rospy.loginfo(f"Successfully rotation presetting")

    def send_modbus_command(self, address, data, slave_id):
        try:
            # サービスリクエストを作成
            request = ModbusWrite()
            request.address = address
            request.data = data
            request.slave_id = slave_id

            # サービスを呼び出す
            response = self.modbus_write_pub.publish(request)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def write_rotation_movement(self, step, slave):
        step_upper, step_lower = self.decimal_to_hex(step)
        # クランプ動作を設定
        self.send_modbus_command(0x005c, [step_upper, step_lower], slave)

    def decimal_to_hex(self, number):
        # Convert the decimal number to a 32-bit signed integer
        number = int(number) & 0xffffffff

        # Convert to hex without formatting into a string
        hex_number = number

        # Split the hex number into two parts and return as integers
        return hex_number >> 16, hex_number & 0xffff


    def move_start(self):
        if self.rotation_status == rotation_status_dict['LEFT']:
            self.write_rotation_movement([0, -1000], self.slave_id)
        elif self.rotation_status == rotation_status_dict['CENTER']:
            self.write_rotation_movement([0, 0], self.slave_id)
        elif self.rotation_status == rotation_status_dict['RIGHT']:
            self.write_rotation_movement([0, 1000], self.slave_id)

    def callback(self, msg):
        if self.rotation_status != msg.data:
            self.rotation_status = msg.data

            # モーターを回転させる
            self.move_start()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RotationPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
