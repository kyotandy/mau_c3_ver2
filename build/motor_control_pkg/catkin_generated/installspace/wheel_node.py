#!/usr/bin/env python3

import threading
import rospy
from std_msgs.msg import Int32MultiArray
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian

class WheelNode:
    def __init__(self):
        self.left_slave_id = 8
        self.right_slave_id = 9

        # Initialize the node
        rospy.init_node('wheel_node', anonymous=True)  
        # Subscribe to the motor speeds topic
        self.speed_sub = rospy.Subscriber("wheel_speeds", Int32MultiArray, self.callback, queue_size=1)

        # Set up Modbus client
        self.client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=115200, parity='E', stopbits=1, bytesize=8, timeout=1)
        
        # Attempt to connect to the Modbus server
        if not self.client.connect():
            rospy.logerr("Failed to connect to Modbus server")
            exit(1)
        
        # Drive trigger -4: speed
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfffc], slave=self.right_slave_id)
        self.client.write_registers(address=0x0066, values=[0xffff, 0xfffc], slave=self.left_slave_id)
        
        # Step
        self.client.write_registers(address=0x005c, values=[0, 100], slave=self.right_slave_id)
        self.client.write_registers(address=0x005c, values=[0, 100], slave=self.left_slave_id)

    def callback(self, data):
        # Extract left and right motor speeds
        left_motor_speed = data.data[0]
        right_motor_speed = data.data[1]

        # Start a new thread for each Modbus write operation
        threading.Thread(target=self.write_motor_speed, args=(right_motor_speed, self.right_slave_id)).start()
        threading.Thread(target=self.write_motor_speed, args=(left_motor_speed, self.left_slave_id)).start()

    def write_motor_speed(self, speed, slave):
        try:
            self.client.write_registers(address=0x005e, values=[0, speed], slave=slave)
            rospy.loginfo(f"Motor speed {speed} sent to slave {slave} via Modbus")
        except Exception as e:
            rospy.logerr(f"Failed to send motor speed via Modbus: {e}")

    def run(self):
        rospy.spin()

    def __del__(self):
        # Close the Modbus client connection when the node is terminated
        if self.client:
            self.client.close()

if __name__ == '__main__':
    wheel_node = WheelNode()
    wheel_node.run()
