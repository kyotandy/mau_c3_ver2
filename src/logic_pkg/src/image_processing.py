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
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback, queue_size=1)

        # Publisher for offset, angle, arc distance, and stop signal data
        self.data_pub = rospy.Publisher("image_info", Float32MultiArray, queue_size=1)

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

        # Define the blue color range for detection (line)
        lower_blue = np.array([100, 50, 0])
        upper_blue = np.array([140, 100, 200])
        # Create mask to detect blue color
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Define the pink color range for detection (stop point)
        lower_pink = np.array([140, 100, 100])
        upper_pink = np.array([170, 255, 255])
        # Create mask to detect pink color
        pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

        # Define the red color range for detection (arc)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        # Create mask to detect red color
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Display the masks in separate windows for debugging
        # cv2.imshow("Blue Mask", blue_mask)
        # cv2.imshow("Purple Mask", pink_mask)
        # cv2.imshow("Green Mask", red_mask)

        # Use the blue mask to find blue regions in the image (line)
        edges = cv2.Canny(blue_mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)

        # Initialize the output values as NaN to indicate detection failure by default
        camera_offset_mm = float('nan')
        camera_angle_deg = float('nan')
        arc_distance_mm = float('nan')
        stop_signal = 0.0

        # Draw only one detected line and calculate offset and angle
        if lines is not None and len(lines) > 0:
            # Use the first detected line
            x1, y1, x2, y2 = lines[0][0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw blue line for the first detected line

            # Calculate the midpoint of the line
            line_mid_x = (x1 + x2) / 2
            line_mid_y = (y1 + y2) / 2

            # Calculate the offset from the center of the image in mm
            offset_x = image_center_x - line_mid_x
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
        
        try:
            # robot_offset_mm = camera_offset_mm - self.camera_tire_gap_mm * math.tan(abs(camera_angle_deg))
            robot_offset_mm = camera_offset_mm
            robot_angle_deg = camera_angle_deg
        except:
            robot_offset_mm = float('nan')
            robot_angle_deg = float('nan')
        
        # Detect red arc and calculate distance from center line
        
        # 画像の中心列から±25ピクセルの範囲を選択（幅50ピクセル）
        start_x = int(image_center_x - 25)
        end_x = int(image_center_x + 25)

        # 範囲内の緑のピクセルを特定
        red_points = np.where(np.any(red_mask[:, start_x:end_x] > 0, axis=1))[0]

        # 緑のピクセルが存在する場合、最も上にある緑のピクセルの位置を取得
        if len(red_points) > 0:
            red_y = red_points[0]
            arc_distance_mm = (red_y - image_center_y) * self.pixel_to_mm


        # Detect pink stop point in 100x100 region around center
        stop_region = pink_mask[int(image_center_y - 50):int(image_center_y + 50),
                                  int(image_center_x - 50):int(image_center_x + 50)]
        if np.any(stop_region > 0):
            stop_signal = 1.0

        # Display the offset, angle, and other information on the image
        text_line1 = f"Offset: {robot_offset_mm:.2f} mm, Angle: {robot_angle_deg:.2f} degrees"
        text_line2 = f"Arc Dist: {arc_distance_mm:.2f} mm, Stop: {stop_signal}"
        cv2.putText(cv_image, text_line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(cv_image, text_line2, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Prepare the data to publish
        data_msg = Float32MultiArray()
        data_msg.data = [robot_offset_mm, robot_angle_deg, arc_distance_mm, stop_signal]
        self.data_pub.publish(data_msg)

        # Display the image with the detected line, offset, angle, and cross
        cv2.imshow("Blue Line Detection", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.run()
