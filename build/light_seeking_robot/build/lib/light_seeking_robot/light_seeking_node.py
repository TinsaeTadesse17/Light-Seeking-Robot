#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LightSeekingNode(Node):
    def __init__(self):
        super().__init__('light_seeking_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_speed', 0.5),
                ('angular_gain', 0.005),
                ('min_contour_area', 100),
                ('max_linear_speed', 1.0),
                ('brightness_threshold', 200)
            ]
        )
        
        self.get_logger().info("Light seeking node initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to grayscale and apply threshold
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 
                                    self.get_parameter('brightness_threshold').value, 
                                    255, 
                                    cv2.THRESH_BINARY)
            
            # Find contours in the threshold image
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                if cv2.contourArea(largest_contour) > self.get_parameter('min_contour_area').value:
                    # Calculate centroid of the largest contour
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Calculate control signals
                        self.calculate_velocity(cx, cy, cv_image.shape[1])
                        return
                        
            # If no valid light source found, stop
            self.stop_robot()
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            self.stop_robot()

    def calculate_velocity(self, x, y, image_width):
        twist = Twist()
        
        # Get parameters
        linear_speed = self.get_parameter('linear_speed').value
        angular_gain = self.get_parameter('angular_gain').value
        max_linear = self.get_parameter('max_linear_speed').value
        
        # Calculate horizontal error (difference from image center)
        error = x - (image_width // 2)
        
        # Set linear velocity (constant forward speed)
        twist.linear.x = min(linear_speed, max_linear)
        
        # Calculate angular velocity (proportional control)
        twist.angular.z = -angular_gain * error
        
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LightSeekingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()