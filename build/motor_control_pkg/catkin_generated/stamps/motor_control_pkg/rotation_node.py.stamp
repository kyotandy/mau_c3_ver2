#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from pymodbus.client import ModbusSerialClient as ModbusClient

rotation_status_dict = {
    'LEFT': '0',
    'CENTER': '1',
    'RIGHT': '2'
}


class RotationPositionNode:
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
        rospy.init_node('rotation_position_node', anonymous=True)
        self.rotation_position_sub = rospy.Subscriber('/rotation_position', String, self.callback, queue_size=1)

        self.rotation_status = rotation_status_dict['CENTER']
        self.slave_id = 1

    def presetting(self):
        # driving data No 1: LEFT
        self.client.write_registers(address=0x0058, values=[
            0, 1, # driving data: 0
            0, 1, # 1: absolute
            0xffff, 0xfd58, # step
            0, 100 # speed
        ])
        # trigger -7: driving data No
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfff9])
        
        # driving data No 2: CENTER
        self.client.write_registers(address=0x0058, values=[
            0, 2, # driving data: 0
            0, 1, # 1: absolute
            0, 0, # step
            0, 100 # speed
        ])
        # trigger -7: driving data No
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfff9])        

        # driving data No 3: RIGHT
        self.client.write_registers(address=0x0058, values=[
            0, 3, # driving data: 0
            0, 1, # 1: absolute
            0, 680, # step
            0, 100 # speed
        ])
        # trigger -7: driving data No
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfff9])        


    def move_start(self):
        if self.rotation_status == rotation_status_dict['LEFT']:
            self.client.write_registers(address=0x0058, values=[0, 1], slave=self.slave_id)
        elif self.rotation_status == rotation_status_dict['CENTER']:
            self.client.write_registers(address=0x0058, values=[0, 2], slave=self.slave_id)
        elif self.rotation_status == rotation_status_dict['RIGHT']:
            self.client.write_registers(address=0x0058, values=[0, 3], slave=self.slave_id)

    def callback(self, msg):
        if self.rail_status != msg.data:
            self.rail_status = msg.data

            self.move_start()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = RotationPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

