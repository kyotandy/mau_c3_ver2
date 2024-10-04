#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
import threading
from pymodbus.client import ModbusSerialClient as ModbusClient

panel_status_dict = {
    'PANEL_ALL_ON': '0',
    'PANEL_RIGHT_FORWARD_OFF': '1',
    'PANEL_RIGHT_BACKWARD_OFF': '2',
    'PANEL_LEFT_FORWARD_OFF': '3',
    'PANEL_LEFT_BACKWARD_OFF': '4'
}


class PanelPositionNode:
    def __init__(self):
        # Modbusクライアントの初期化
        self.client = ModbusClient(
            method='rtu',
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=3,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        self.client.connect()
        self.presetting()

        # ROSノードの初期化
        rospy.init_node('panel_position_node', anonymous=True)
        self.panel_position_sub = rospy.Subscriber('/panel_position', String, self.callback)

        self.panel_status = panel_status_dict['ALL_ON']
        self.head_right_forward_slave_id = 5
        self.head_right_backward_slave_id = 6
        self.head_left_forward_slave_id = 7
        self.head_left_backward_slave_id = 8
        self.tail_right_forward_slave_id = 11
        self.tail_right_backward_slave_id = 12
        self.tail_left_forward_slave_id = 13
        self.tail_left_backward_slave_id = 14

    def presetting(self):
        # driving data No 0: OFF
        self.client.write_registers(address=0x0058, values=[
            0, 0, # driving data: 0
            0, 1, # 1: absolute
            0, 1000, # step
            0, 100 # speed
        ])
        # trigger -7: driving data No
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfff9])
        
        # driving data No 1: ON
        self.client.write_registers(address=0x0058, values=[
            0, 1, # driving data: 0
            0, 1, # 1: absolute
            0, 0, # step
            0, 100 # speed
        ])
        # trigger -7: driving data No
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfff9])        

    def write_panel_movement(self, driving_no, slave):
        try:
            self.client.write_registers(address=0x0058, values=[0, driving_no], slave=slave)
            rospy.loginfo(f"Motor speed {driving_no} sent to slave {slave} via Modbus")
        except Exception as e:
            rospy.logerr(f"Failed to send motor speed via Modbus: {e}")
    
    def start_move(self):
        if self.panel_status == panel_status_dict['ALL_ON']:
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

