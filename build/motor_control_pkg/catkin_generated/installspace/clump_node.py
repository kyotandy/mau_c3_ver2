#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
import threading
from pymodbus.client import ModbusSerialClient as ModbusClient

clump_status_dict = {
    'ALL_OFF': '0',
    'FORWARD_ON': '1',
    'BACKWARD_ON': '2',
    'ALL_ON': '3'
}


class ClumpPositionNode:
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
        rospy.init_node('clump_position_node', anonymous=True)
        self.clump_position_sub = rospy.Subscriber('/clump_position', String, self.callback)

        self.clump_status = clump_status_dict['ALL_OFF']
        self.forward_slave_id = 2
        self.backward_slave_id = 3

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

    def write_clump_movement(self, driving_no, slave):
        try:
            self.client.write_registers(address=0x0058, values=[0, driving_no], slave=slave)
            rospy.loginfo(f"Motor speed {driving_no} sent to slave {slave} via Modbus")
        except Exception as e:
            rospy.logerr(f"Failed to send motor speed via Modbus: {e}")
    
    def start_move(self):
        if self.clump_status == clump_status_dict['ALL_OFF']:
            threading.Thread(target=self.write_clump_movement, args=(0, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(0, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['FORWARD_ON']:
            threading.Thread(target=self.write_clump_movement, args=(1, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(0, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['BACKWARD_ON']:
            threading.Thread(target=self.write_clump_movement, args=(0, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(1, self.backward_slave_id)).start()
        elif self.clump_status == clump_status_dict['ALL_ON']:
            threading.Thread(target=self.write_clump_movement, args=(1, self.forward_slave_id)).start()
            threading.Thread(target=self.write_clump_movement, args=(1, self.backward_slave_id)).start()

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

