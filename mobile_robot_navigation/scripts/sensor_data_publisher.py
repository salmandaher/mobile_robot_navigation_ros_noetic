#!/usr/bin/env python3

import rospy
from mobile_robot_navigation.msg import SensorData
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import random
import math

class SensorDataPublisher:
    def __init__(self):
        rospy.init_node('sensor_data_publisher', anonymous=True)

        # Publisher for custom sensor data
        self.sensor_pub = rospy.Publisher('/sensor_data', SensorData, queue_size=10)

        # Publishers for Range messages for front, left, and right sensors
        self.range_pub_front = rospy.Publisher('/sensor_range_front', Range, queue_size=10)
        self.range_pub_left = rospy.Publisher('/sensor_range_left', Range, queue_size=10)
        self.range_pub_right = rospy.Publisher('/sensor_range_right', Range, queue_size=10)

        # Subscriber to get robot position for realistic sensor simulation
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Parameters
        self.max_range = rospy.get_param('~max_range', 4.0)  # Maximum sensor range in meters
        self.min_range = rospy.get_param('~min_range', 0.04) # Minimum sensor range in meters
        self.noise_std = rospy.get_param('~noise_std', 0.02) # Noise standard deviation
        self.publish_rate = rospy.get_param('~publish_rate', 10) # Hz

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Environment obstacles
        self.wall_x = 3.0  # Wall position at x=3.0
        self.obstacle_x = 1.5 + 0.5 * math.sin(rospy.get_time() * 0.3)  # Moving obstacle
        self.obstacle_y = 0.5 + 0.3 * math.cos(rospy.get_time() * 0.2)

        self.rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("Sensor Data Publisher initialized - Realistic environment simulation")

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        import tf.transformations
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def add_noise(self, value):
        """Add Gaussian noise to sensor reading"""
        noise = random.gauss(0, self.noise_std)
        return max(self.min_range, min(self.max_range, value + noise))

    def calculate_distance_to_wall(self, sensor_angle):
        """Calculate distance from robot to wall considering sensor direction"""
        # Wall is vertical at x = wall_x
        sensor_global_angle = self.robot_theta + sensor_angle
        
        if abs(math.cos(sensor_global_angle)) < 0.01:  # Nearly parallel to wall
            return self.max_range
            
        # Distance to wall along sensor ray
        dx = self.wall_x - self.robot_x
        distance_to_wall = dx / math.cos(sensor_global_angle)
        
        if distance_to_wall <= 0:  # Wall is behind
            return self.max_range
            
        return distance_to_wall

    def calculate_distance_to_obstacle(self, sensor_angle):
        """Calculate distance from robot to moving obstacle"""
        current_time = rospy.get_time()
        # Update obstacle position
        self.obstacle_x = 1.5 + 0.5 * math.sin(current_time * 0.3)
        self.obstacle_y = 0.5 + 0.3 * math.cos(current_time * 0.2)
        
        sensor_global_angle = self.robot_theta + sensor_angle
        
        # Vector from robot to obstacle
        dx = self.obstacle_x - self.robot_x
        dy = self.obstacle_y - self.robot_y
        
        # Distance and angle to obstacle
        distance_to_obstacle = math.sqrt(dx*dx + dy*dy)
        angle_to_obstacle = math.atan2(dy, dx)
        
        # Check if obstacle is in sensor direction (within field of view)
        angle_diff = abs(angle_to_obstacle - sensor_global_angle)
        if angle_diff > math.pi:
            angle_diff = 2*math.pi - angle_diff
            
        # If obstacle is within sensor field of view (assuming 30 degree FOV)
        if angle_diff < 0.26:  # 30 degrees in radians
            return distance_to_obstacle
        else:
            return self.max_range

    def simulate_environment_sensors(self):
        """Simulate realistic sensor readings based on robot position and environment"""
        # Sensor angles relative to robot (front, left, right)
        front_angle = 0.0
        left_angle = math.pi/2  # 90 degrees left
        right_angle = -math.pi/2  # 90 degrees right
        
        # Calculate distances to environmental features
        front_wall_dist = self.calculate_distance_to_wall(front_angle)
        left_wall_dist = self.calculate_distance_to_wall(left_angle)
        right_wall_dist = self.calculate_distance_to_wall(right_angle)
        
        front_obs_dist = self.calculate_distance_to_obstacle(front_angle)
        left_obs_dist = self.calculate_distance_to_obstacle(left_angle)
        right_obs_dist = self.calculate_distance_to_obstacle(right_angle)
        
        # Take minimum distance (closest obstacle)
        front_range = min(front_wall_dist, front_obs_dist)
        left_range = min(left_wall_dist, left_obs_dist)
        right_range = min(right_wall_dist, right_obs_dist)
        
        # Add noise to readings
        front_range = self.add_noise(front_range)
        left_range = self.add_noise(left_range)
        right_range = self.add_noise(right_range)

        return front_range, left_range, right_range

    def publish_range_message(self, range_val, frame_id, publisher):
        """Publish a sensor_msgs/Range message for a given sensor"""
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.52  # 30 degrees in radians
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        range_msg.range = range_val
        publisher.publish(range_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Create sensor data message
            sensor_msg = SensorData()
            sensor_msg.header = Header()
            sensor_msg.header.stamp = rospy.Time.now()
            sensor_msg.header.frame_id = "base_link"

            # Simulate realistic sensor readings
            front_range, left_range, right_range = self.simulate_environment_sensors()

            sensor_msg.range_front = front_range
            sensor_msg.range_left = left_range
            sensor_msg.range_right = right_range
            sensor_msg.battery_voltage = 12.0 + random.uniform(-0.5, 0.5)
            sensor_msg.obstacle_detected = front_range < 0.5

            # Publish sensor data
            self.sensor_pub.publish(sensor_msg)

            # Publish Range messages for RViz visualization
            self.publish_range_message(front_range, "front_sensor_link", self.range_pub_front)
            self.publish_range_message(left_range, "left_sensor_link", self.range_pub_left)
            self.publish_range_message(right_range, "right_sensor_link", self.range_pub_right)

            rospy.logdebug(f"Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) - Sensors: F={front_range:.2f}, L={left_range:.2f}, R={right_range:.2f}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = SensorDataPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass