#!/usr/bin/env python3

import threading
import rospy
from std_msgs.msg import Int32MultiArray
from motor_control_pkg.srv import ModbusWrite, ModbusWriteRequest

class WheelNode:
    def __init__(self):
        self.left_slave_id = 9
        self.right_slave_id = 10

        # Initialize the node
        rospy.init_node('wheel_node', anonymous=True)
        # Subscribe to the motor speeds topic
        self.speed_sub = rospy.Subscriber("wheel_speeds", Int32MultiArray, self.callback, queue_size=1)

        # Wait for the modbus_write service to be available
        rospy.wait_for_service('modbus_write')

        # Set up the service proxy for Modbus communication
        self.modbus_write_service = rospy.ServiceProxy('modbus_write', ModbusWrite)

        # Initialize the motors with the preset values
        self.pre_setting()

    def pre_setting(self):
        # Drive trigger -4: speed
        self.send_modbus_command(0x0066, [0xffff, 0xfffc], self.right_slave_id)
        self.send_modbus_command(0x0066, [0xffff, 0xfffc], self.left_slave_id)
        
        # Step
        self.send_modbus_command(0x005c, [0, 50], self.right_slave_id)
        self.send_modbus_command(0x005c, [0, 50], self.left_slave_id)

        rospy.loginfo(f"Successfully wheel presetting")

    def write_wheel_movement(self, speed, slave):
        speed_upper, speed_lower = self.decimal_to_hex(speed)
        # クランプ動作を設定
        self.send_modbus_command(0x005e, [speed_upper, speed_lower], slave)

    def decimal_to_hex(self, number):
        # Convert the decimal number to a 32-bit signed integer
        number = int(number) & 0xffffffff

        # Convert to hex without formatting into a string
        hex_number = number

        # Split the hex number into two parts and return as integers
        return hex_number >> 16, hex_number & 0xffff
        

    def callback(self, data):
        # Extract left and right motor speeds
        left_motor_speed = data.data[0]
        right_motor_speed = data.data[1]

        # Start a new thread for each Modbus write operation
        threading.Thread(target=self.write_wheel_movement, args=(right_motor_speed, self.right_slave_id)).start()
        threading.Thread(target=self.write_wheel_movement, args=(left_motor_speed, self.left_slave_id)).start()

    def send_modbus_command(self, address, data, slave_id):
        try:
            # Create a service request
            request = ModbusWriteRequest()
            request.address = address
            request.data = data
            request.slave_id = slave_id

            # Call the service
            response = self.modbus_write_service(request)

            rospy.loginfo(f"send command to {address}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        wheel_node = WheelNode()
        wheel_node.run()
    except rospy.ROSInterruptException:
        pass
