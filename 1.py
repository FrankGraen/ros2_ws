# ros_scripts/line_follower.py

'''
This script accepts the content taken
by the live camera (model ORBBEC Astra Pro Plus)
and displays it one the computure screen, 
with the subscribed topic /image_raw, in
addition to the fact that the script also takes
on the function of processing the image to determine
the direction of travel of the trolley.

Before running this script, make sure you run the
following command in the terminal:

ros2 run usb_cam usb_cam_node_exe

to enable the camera feature.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import numpy as np

class LineFollower(Node):
    
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10  # QoS profile
        )
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the image (for debugging purposes)
        # cv2.imshow("Camera View", cv_image)
        # cv2.waitKey(1)
        
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Add code for line following logic using OpenCV
        # e.g., processing the image to detect the line
        
        # Define the red color range in HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        # Combine masks
        mask = mask1 + mask2
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours are found, find the largest one and calculate its centroid
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Draw the centroid on the image
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                
                # Example of steering logic based on centroid position
                image_width = cv_image.shape[1]
                if cx < image_width // 3:
                    # Turn left
                    self.get_logger().info('Turning left')
                elif cx > 2 * image_width // 3:
                    # Turn right
                    self.get_logger().info('Turning right')
                else:
                    # Go straight
                    self.get_logger().info('Going straight')
        
        # Display the processed image
        cv2.imshow('Red Line Detection', cv_image)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    
    # Destroy the node explicitly
    line_follower.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()