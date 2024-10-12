#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from motor_control_pkg.msg import ModbusWrite

rail_status_dict = {
    'RAIL_OFF': '0',
    'RAIL_ON': '1'
}

class RailNode:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('rail_positioning_node', anonymous=True)
        self.rail_moving_info_subscriber = rospy.Subscriber('/rail_position', String, self.callback, queue_size=1)
        self.modbus_write_pub = rospy.Publisher('/modbus_request', ModbusWrite, queue_size=1)


        self.rail_status = rail_status_dict['RAIL_OFF']
        self.slave_id = 4

        # 初期設定
        self.presetting()

    def presetting(self):
        # モーターの励磁をオンにする
        self.send_modbus_command(0x001e, [0x2000], self.slave_id)

        # driving No 0: OFF
        # step(0)
        self.send_modbus_command(0x0402, [0, 0], self.slave_id)
        # speed(500)
        self.send_modbus_command(0x0502, [0, 500], self.slave_id)
        # type(absolute)
        self.send_modbus_command(0x0601, [1], self.slave_id)

        # dribing No 1: ON
        # step(1000)
        self.send_modbus_command(0x0404, [0, 1000], self.slave_id)
        # speed(500)
        self.send_modbus_command(0x0504, [0, 500], self.slave_id)
        # type(absolute)
        self.send_modbus_command(0x0602, [1], self.slave_id)

        rospy.loginfo(f"Successfully rail position presetting")

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

    def move_start(self):
        if self.rail_status == rail_status_dict['RAIL_ON']:
            self.send_modbus_command(0x001e, [0x2101], self.slave_id)
        elif self.rail_status == rail_status_dict['RAIL_OFF']:
            self.send_modbus_command(0x001e, [0x2102], self.slave_id)
        
        # motor stop command
        self.send_modbus_command(0x001e, [0x2001], self.slave_id)

    def callback(self, msg):
        if self.rail_status != msg.data:
            self.rail_status = msg.data

            # 移動開始
            self.move_start()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RailNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
