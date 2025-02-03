#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Tuned parameters
        self.max_speed = 0.6
        self.min_speed = 0.1
        self.max_yaw_rate = 2.0
        self.max_accel = 0.3
        self.predict_time = 2.0
        self.robot_radius = 1.0
        self.safety_distance = 0.8
        self.last_movement_time = time.time()

        # State
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.current_vel = Twist()

        # Subscribers/Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/lidar/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_vel = msg.twist.twist
        
        # Update movement timestamp
        if abs(self.current_vel.linear.x) > 0.01 or abs(self.current_vel.angular.z) > 0.01:
            self.last_movement_time = time.time()

    def quaternion_to_yaw(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        return np.arctan2(2*(w*z + x*y), 1-2*(y**2 + z**2))

    def scan_callback(self, msg):
        # Process scan data
        obstacles = []
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                global_x = self.current_pose[0] + r * np.cos(angles[i] + self.current_pose[2])
                global_y = self.current_pose[1] + r * np.sin(angles[i] + self.current_pose[2])
                obstacles.append((global_x, global_y))

        # Stuck recovery
        if self.is_stuck():
            self.get_logger().warn("Executing stuck recovery!")
            cmd = Twist()
            cmd.angular.z = self.max_yaw_rate * 1.5
            self.cmd_vel_pub.publish(cmd)
            return

        # Generate control command
        cmd = self.dwa_control(self.current_pose, self.current_vel, obstacles)
        self.cmd_vel_pub.publish(cmd)

    def is_stuck(self):
        return (time.time() - self.last_movement_time) > 5.0  # 5 seconds no movement

    def dwa_control(self, pose, vel, obstacles):
        dw = [
            max(self.min_speed, vel.linear.x - self.max_accel * self.predict_time),
            min(self.max_speed, vel.linear.x + self.max_accel * self.predict_time),
            max(-self.max_yaw_rate, vel.angular.z - self.max_yaw_rate * self.predict_time),
            min(self.max_yaw_rate, vel.angular.z + self.max_yaw_rate * self.predict_time)
        ]
        
        best_score = -float('inf')
        best_cmd = Twist()
        
        # Immediate collision check
        current_dist = self.calc_clearance([pose[:2]], obstacles)
        if current_dist < self.safety_distance:
            best_cmd.angular.z = self.max_yaw_rate
            return best_cmd

        # Sample possible velocities
        for v in np.linspace(dw[0], dw[1], 15):
            for w in np.linspace(dw[2], dw[3], 15):
                if v < 0:  # No reversing
                    continue
                
                traj = self.predict_trajectory(pose, v, w)
                clearance = self.calc_clearance(traj, obstacles)
                
                # Scoring criteria
                speed_score = v / self.max_speed
                clearance_score = clearance / 5.0  # Normalize
                heading_score = 1.0  # Simple forward preference
                
                total_score = (
                    0.5 * clearance_score +
                    0.3 * speed_score +
                    0.2 * heading_score
                )
                
                # Update best command
                if clearance > self.safety_distance and total_score > best_score:
                    best_score = total_score
                    best_cmd.linear.x = v
                    best_cmd.angular.z = w

        # Fallback behavior
        if best_score == -float('inf'):
            best_cmd.linear.x = 0.0
            best_cmd.angular.z = self.max_yaw_rate * 0.75
            
        return best_cmd

    def predict_trajectory(self, pose, v, w):
        dt = 0.1
        steps = int(self.predict_time / dt)
        x, y, theta = pose
        traj = []
        
        for _ in range(steps):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
            traj.append((x, y))
            
        return traj

    def calc_clearance(self, traj, obstacles):
        if not obstacles:
            return float('inf')
            
        min_dist = min(
            np.hypot(p[0]-ox, p[1]-oy)
            for p in traj
            for (ox, oy) in obstacles
        )
        return min_dist - self.robot_radius

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()