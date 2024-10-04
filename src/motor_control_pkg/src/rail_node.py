#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from pymodbus.client import ModbusSerialClient as ModbusClient

rail_status_dict = {
    'RAIL_OFF': '0',
    'RAIL_ON': '1'
}


class RailPositioningNode:
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
        rospy.init_node('rail_positioning_node', anonymous=True)
        self.rail_moving_info_subscriber = rospy.Subscriber('/rail_position', String, self.callback)

        self.rail_status = rail_status_dict['RAIL_ON']
        self.slave_id = 4

    def presetting(self):
        # Turn on motor excitation
        self.client.write_register(address=0x001e, value=0x2000, slave=self.slave_id)

        # define driving detail for driving No.1
        self.client.write_registers(address=0x0402, values=[0, 3000], slave=self.slave_id)
        self.client.write_registers(address=0x0502, values=[0, 500], slave=self.slave_id)
        self.client.write_register(address=0x0601, value=1, slave=self.slave_id)

        # define driving detail for driving No.2
        self.client.write_registers(address=0x0404, values=[0, 0], slave=self.slave_id)
        self.client.write_registers(address=0x0504, values=[0, 500], slave=self.slave_id)
        self.client.write_register(address=0x0602, value=1, slave=self.slave_id)

    def move_start(self):
        if self.rail_status == rail_status_dict['RAIL_ON']:
            self.client.write_register(address=0x001e, value=0x2101, slave=self.slave_id)
        elif self.rail_status == rail_status_dict['RAIL_OFF']:
            self.client.write_registers(address=0x001e, values=0x2102, slave=self.slave_id)

    def callback(self, msg):
        if self.rail_status != msg.data:
            self.rail_status = msg.data

            # motor stop
            self.client.write_register(address=0x001e, value=0x2001, slave=self.slave_id)

            self.move_start()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = RailPositioningNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

