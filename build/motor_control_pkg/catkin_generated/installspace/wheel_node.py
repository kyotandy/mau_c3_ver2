#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int32MultiArray
from motor_control_pkg.msg import ModbusWrite

class WheelNode:
    def __init__(self):
        self.left_slave_id = 9
        self.right_slave_id = 10

        # Initialize the node
        rospy.init_node('wheel_node', anonymous=True)
        # Subscribe to the motor speeds topic
        self.speed_sub = rospy.Subscriber("wheel_speeds", Int32MultiArray, self.callback, queue_size=1)
        self.modbus_write_pub = rospy.Publisher('modbus_request', ModbusWrite, queue_size=1)

        # Initialize the motors with the preset values
        self.pre_setting()

    def pre_setting(self):
        # Drive trigger -4: speed
        self.send_modbus_command(0x0066, [0xffff, 0xfffc], self.left_slave_id)
        time.sleep(1)
        self.send_modbus_command(0x0066, [0xffff, 0xfffc], self.right_slave_id)
        time.sleep(1)
        
        # Step
        self.send_modbus_command(0x005c, [0xffff, 0xfe0c, 0, 500], self.left_slave_id)
        time.sleep(1)
        self.send_modbus_command(0x005c, [0xffff, 0xfe0c, 0, 500], self.right_slave_id)
        time.sleep(3)

        self.send_modbus_command(0x005c, [0, 500, 0, 500], self.left_slave_id)
        time.sleep(1)
        self.send_modbus_command(0x005c, [0, 500, 0, 500], self.right_slave_id)
        time.sleep(1)

        

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
        rospy.loginfo(f'receive left_motor_speed: {data}')

        # Start a new thread for each Modbus write operation
        self.write_wheel_movement(right_motor_speed, self.right_slave_id)
        time.sleep(0.1)
        self.write_wheel_movement(left_motor_speed, self.left_slave_id)

        time.sleep(0.1)

    def send_modbus_command(self, address, data, slave_id):
        try:
            # Create a service request
            request = ModbusWrite()
            request.address = address
            request.data = data
            request.slave_id = slave_id

            # Call the service
            self.modbus_write_pub.publish(request)

            rospy.loginfo(f"send command to {data[0], data[1]}")

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
