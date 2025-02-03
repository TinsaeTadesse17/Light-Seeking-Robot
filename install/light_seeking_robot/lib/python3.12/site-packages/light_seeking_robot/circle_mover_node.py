#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)  
        self.declare_parameter('radius', 5.0)       
        self.declare_parameter('publish_rate', 10.0) 

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.radius = self.get_parameter('radius').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Calculate angular velocity (ω = v/r)
        self.angular_speed = self.linear_speed / self.radius

        # Create publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_cmd)
        

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.cmd_pub.publish(msg)
        self.get_logger().debug(f'Publishing cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}', throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when exiting
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()