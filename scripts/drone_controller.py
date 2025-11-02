#!/usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi

class DroneController:
    def __init__(self, drone_id, target_area):
        self.drone_id = drone_id
        self.target_area = target_area
        self.current_pose = None
        self.target_reached = False
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher(f'/drone_{drone_id}/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(f'/drone_{drone_id}/odom', Odometry, self.odom_callback)
        
        rospy.loginfo(f"Drone {drone_id} controller initialized. Target: {target_area}")
    
    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_pose = msg.pose.pose
    
    def get_distance_to_target(self, target_x, target_y):
        """Calculate distance to target position"""
        if self.current_pose is None:
            return float('inf')
        
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        return sqrt(dx*dx + dy*dy)
    
    def get_angle_to_target(self, target_x, target_y):
        """Calculate angle to target position"""
        if self.current_pose is None:
            return 0
        
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        return atan2(dy, dx)
    
    def move_to_target(self, target_x, target_y, target_z):
        """Move drone towards target position"""
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo(f"Drone {self.drone_id} moving to area {self.target_area}: ({target_x}, {target_y}, {target_z})")
        
        while not rospy.is_shutdown() and not self.target_reached:
            if self.current_pose is None:
                rate.sleep()
                continue
            
            distance = self.get_distance_to_target(target_x, target_y)
            
            if distance < 0.5:  # Reached target (within 0.5m)
                # Stop the drone
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                self.target_reached = True
                rospy.loginfo(f"Drone {self.drone_id} reached area {self.target_area}!")
                break
            
            # Calculate velocity command
            cmd = Twist()
            
            # Linear velocity (proportional to distance)
            max_linear_vel = 2.0
            cmd.linear.x = min(max_linear_vel, distance * 0.5)
            
            # For planar_move plugin, we need to use angular.z for turning
            # and the drone will move in the direction it's facing
            angle_to_target = self.get_angle_to_target(target_x, target_y)
            
            # Simple proportional control for rotation
            cmd.angular.z = angle_to_target * 0.5
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()


def main():
    rospy.init_node('drone_controller_node')
    
    # Load configuration
    config_path = rospy.get_param('~config_path', 
                                   os.path.join(os.path.dirname(__file__), '../config/areas.yaml'))
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    num_drones = config['num_drones']
    areas = config['areas']
    area_list = list(areas.keys())
    
    # Wait for simulation to start
    rospy.sleep(5.0)
    rospy.loginfo("Starting drone navigation...")
    
    # Create controllers for each drone
    controllers = []
    for i in range(num_drones):
        # Assign drones to areas (round-robin if more drones than areas)
        target_area = area_list[i % len(area_list)]
        controller = DroneController(i, target_area)
        controllers.append(controller)
    
    # Wait for all drones to get odometry
    rospy.sleep(2.0)
    
    # Start moving drones to their target areas
    rospy.loginfo("Moving drones to target areas...")
    
    for controller in controllers:
        area_config = areas[controller.target_area]
        target_x = area_config['x']
        target_y = area_config['y']
        target_z = area_config['z']
        
        # Start moving (this will block for each drone sequentially)
        # For simultaneous movement, you'd need to use threading
        controller.move_to_target(target_x, target_y, target_z)
    
    rospy.loginfo("All drones have reached their target areas!")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
