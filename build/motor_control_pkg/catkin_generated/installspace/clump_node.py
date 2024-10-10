#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading
from motor_control_pkg.srv import ModbusWrite, ModbusWriteRequest

clump_status_dict = {
    'ALL_OFF': '0',
    'FORWARD_ON': '1',
    'BACKWARD_ON': '2',
    'ALL_ON': '3'
}

class ClumpPositionNode:
    def __init__(self):
        self.clump_status = clump_status_dict['ALL_OFF']
        self.forward_slave_id = 2
        self.backward_slave_id = 3

        # ROSノードの初期化
        rospy.init_node('clump_position_node', anonymous=True)
        self.clump_position_sub = rospy.Subscriber('/clump_position', String, self.callback, queue_size=1)

        # サービスが利用可能になるのを待つ
        rospy.wait_for_service('modbus_write')
        
        # Modbus書き込みサービスのプロキシを作成
        self.modbus_write_service = rospy.ServiceProxy('modbus_write', ModbusWrite)

        # 初期設定
        self.presetting()

    def presetting(self):
        # type(absolute), speed(500), trigger(step)
        self.send_modbus_command(0x0058, [0, 0, 0, 1, 0, 0, 0, 500], self.forward_slave_id)
        self.send_modbus_command(0x0066, [0xffff, 0xfffb], self.forward_slave_id)

        # 同様にbackward_slave_idにも設定
        self.send_modbus_command(0x0058, [0, 0, 0, 1, 0, 0, 0, 500], self.backward_slave_id)
        self.send_modbus_command(0x0066, [0xffff, 0xfffb], self.backward_slave_id)

        rospy.loginfo(f"Successfully clumper presetting")


    def send_modbus_command(self, address, data, slave_id):
        try:
            # サービスリクエストを作成
            request = ModbusWriteRequest()
            request.address = address
            request.data = data
            request.slave_id = slave_id

            # サービスを呼び出す
            response = self.modbus_write_service(request)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def write_clump_movement(self, step, slave):
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

    def start_move(self):
        if self.clump_status == clump_status_dict['ALL_OFF']:
            threading.Thread(target=self.write_clump_movement, args=(1500, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(1500, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['FORWARD_ON']:
            threading.Thread(target=self.write_clump_movement, args=(0, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(1500, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['BACKWARD_ON']:
            threading.Thread(target=self.write_clump_movement, args=(1500, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(0, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['ALL_ON']:
            threading.Thread(target=self.write_clump_movement, args=(0, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(0, self.backward_slave_id)).start()

    def callback(self, msg):
        if self.clump_status != msg.data:
            self.clump_status = msg.data
            self.start_move()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ClumpPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
