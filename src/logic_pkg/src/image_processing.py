#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class ImageSubscriber:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_processing_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback)

        # Publisher for offset and angle data
        self.data_pub = rospy.Publisher("line_offset_angle", Float32MultiArray, queue_size=10)

        # Pixel to mm conversion factor (1 pixel = 0.05 mm)
        self.pixel_to_mm = 0.05
        self.camera_tire_gap_mm = 100

    def callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Get the image dimensions
        image_height, image_width = cv_image.shape[:2]
        image_center_x = image_width / 2
        image_center_y = image_height / 2

        # Draw a cross at the center of the image
        cross_color = (0, 255, 255)  # Yellow color for the cross
        cv2.line(cv_image, (int(image_center_x), 0), (int(image_center_x), image_height), cross_color, 1)
        cv2.line(cv_image, (0, int(image_center_y)), (image_width, int(image_center_y)), cross_color, 1)

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the blue color range for detection
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Create mask to detect blue color
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Use the mask to find blue regions in the image
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)

        camera_offset_mm = 0
        camera_angle_deg = 0

        # Draw only one detected line and calculate offset and angle
        if lines is not None and len(lines) > 0:
            # Use the first detected line
            x1, y1, x2, y2 = lines[0][0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw blue line for the first detected line

            # Calculate the midpoint of the line
            line_mid_x = (x1 + x2) / 2
            line_mid_y = (y1 + y2) / 2

            # Calculate the offset from the center of the image in mm
            offset_x = line_mid_x - image_center_x
            camera_offset_mm = offset_x * self.pixel_to_mm

            # Calculate the angle of the line with respect to the y-axis
            if (y2 - y1) != 0:
                camera_angle_rad = math.atan2(x2 - x1, y2 - y1)  # Swap x and y for angle with respect to y-axis
                # Convert to degrees and adjust sign for clockwise/anti-clockwise convention
                camera_angle_deg = math.degrees(camera_angle_rad)
                # Normalize angle to range [-90, 90]
                if camera_angle_deg > 90:
                    camera_angle_deg -= 180
                elif camera_angle_deg < -90:
                    camera_angle_deg += 180
            
            # calculate robot position offset and angle
            robot_offset_mm = camera_offset_mm - self.camera_tire_gap_mm * math.tan(abs(camera_angle_deg))
            robot_angle_deg = camera_angle_deg

            # Display the offset and angle on the image
            text = f"Offset: {robot_offset_mm:.2f} mm, Angle: {robot_angle_deg:.2f} degrees"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Prepare the data to publish
            data_msg = Float32MultiArray()
            data_msg.data = [robot_offset_mm, robot_angle_deg]
            self.data_pub.publish(data_msg)

        # Display the image with the detected line, offset, angle, and cross
        cv2.imshow("Blue Line Detection", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.run()
