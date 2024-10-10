#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time
import threading
from motor_control_pkg.srv import ModbusWrite, ModbusWriteRequest

panel_status_dict = {
    'PANEL_ALL_ON': '0',
    'PANEL_RIGHT_FORWARD_OFF': '1',
    'PANEL_RIGHT_BACKWARD_OFF': '2',
    'PANEL_LEFT_FORWARD_OFF': '3',
    'PANEL_LEFT_BACKWARD_OFF': '4'
}

class PanelPositionNode:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('panel_position_node', anonymous=True)
        self.panel_position_sub = rospy.Subscriber('/panel_position', String, self.callback, queue_size=1)

        # サービスが利用可能になるのを待つ
        rospy.wait_for_service('modbus_write')
        
        # Modbus書き込みサービスのプロキシを作成
        self.modbus_write_service = rospy.ServiceProxy('modbus_write', ModbusWrite)

        self.panel_status = panel_status_dict['PANEL_ALL_ON']
        self.head_right_forward_slave_id = 5
        self.head_right_backward_slave_id = 6
        self.head_left_forward_slave_id = 7
        self.head_left_backward_slave_id = 8
        self.tail_right_forward_slave_id = 11 
        self.tail_right_backward_slave_id = 12
        self.tail_left_forward_slave_id = 13
        self.tail_left_backward_slave_id = 14

        self.presetting()

    def presetting(self):
        """rospy.loginfo("presetting start")
        
        # driving data No 1: ON
        # step(0)
        self.send_modbus_command(0x0404, [0, 1000], self.head_parent_slave_id)
        time.sleep(0.1)
        self.send_modbus_command(0x0404, [0, 1000], self.tail_parent_slave_id)
        time.sleep(0.1)
        # speed(500)
        self.send_modbus_command(0x0504, [0, 500], self.head_parent_slave_id)
        time.sleep(0.1)
        self.send_modbus_command(0x0504, [0, 500], self.tail_parent_slave_id)
        time.sleep(0.1)
        # type(absolute)
        self.send_modbus_command(0x0602, [1], self.head_parent_slave_id)
        time.sleep(0.1)
        self.send_modbus_command(0x0602, [1], self.tail_parent_slave_id)
        time.sleep(0.1)
        rospy.loginfo("driving data 1 set")

        rospy.loginfo(f"Successfully panel position presetting")
        """
        # driving data No 0: OFF
        # step(0)
        self.send_modbus_command(0x0402, [0, 0], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0402, [0, 0], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0402, [0, 0], self.tail_left_forward_slave_id)
        self.send_modbus_command(0x0402, [0, 0], self.tail_left_backward_slave_id)
        # speed(500)
        self.send_modbus_command(0x0502, [0, 500], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0502, [0, 500], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0502, [0, 500], self.head_left_forward_slave_id)
        self.send_modbus_command(0x0502, [0, 500], self.head_left_backward_slave_id)
        # type(absolute)
        self.send_modbus_command(0x0601, [1], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0601, [1], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0601, [1], self.head_left_forward_slave_id)
        self.send_modbus_command(0x0601, [1], self.head_left_backward_slave_id)
        rospy.loginfo("driving data 0 set")

        # driving data No 1: ON
        # step(0)
        self.send_modbus_command(0x0404, [0, 1000], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0404, [0, 1000], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0404, [0, 1000], self.head_left_forward_slave_id)
        self.send_modbus_command(0x0404, [0, 1000], self.head_left_backward_slave_id)
        # speed(500)
        self.send_modbus_command(0x0504, [0, 500], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0504, [0, 500], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0504, [0, 500], self.head_left_forward_slave_id)
        self.send_modbus_command(0x0504, [0, 500], self.head_left_backward_slave_id)
        # type(absolute)
        self.send_modbus_command(0x0602, [1], self.head_right_forward_slave_id)
        self.send_modbus_command(0x0602, [1], self.head_right_backward_slave_id)
        self.send_modbus_command(0x0602, [1], self.head_left_forward_slave_id)
        self.send_modbus_command(0x0602, [1], self.head_left_backward_slave_id)
        rospy.loginfo("driving data 0 set")
        rospy.loginfo("Successfully clumper presetting")


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

    def write_panel_movement(self, driving_no, slave):
        if driving_no == 0:
            self.send_modbus_command(0x001e, [0x2101], slave)
        elif driving_no == 1:
            self.send_modbus_command(0x001e, [0x2102], slave)
        
        # motor stop command
        self.send_modbus_command(0x001e, [0x2001], slave)

    def start_move(self):
        if self.panel_status == panel_status_dict['PANEL_ALL_ON']:
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_backward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_backward_slave_id)).start()
        elif self.panel_status == panel_status_dict['PANEL_RIGHT_FORWARD_OFF']:
            threading.Thread(target=self.write_panel_movement, args=(0, self.head_right_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_backward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_backward_slave_id)).start()
        elif self.panel_status == panel_status_dict['PANEL_RIGHT_BACKWARD_OFF']:
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(0, self.head_right_backward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_backward_slave_id)).start()
        elif self.panel_status == panel_status_dict['PANEL_LEFT_FORWARD_OFF']:
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_backward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(0, self.head_left_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_backward_slave_id)).start()
        elif self.panel_status == panel_status_dict['PANEL_LEFT_BACKWARD_OFF']:
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_right_backward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(1, self.head_left_forward_slave_id)).start()
            threading.Thread(target=self.write_panel_movement, args=(0, self.head_left_backward_slave_id)).start()

    def callback(self, msg):
        if self.panel_status != msg.data:
            self.panel_status = msg.data
            self.start_move()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PanelPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
