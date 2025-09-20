#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist
from mobile_robot_navigation.msg import SensorData
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import threading
import time
import random
import math

class RosserialMock:
    """Mock Arduino node that simulates rosserial communication and publishes environment markers"""
    
    def __init__(self):
        rospy.init_node('rosserial_mock', anonymous=True)

        # Publishers (Arduino -> ROS)
        self.sensor_pub = rospy.Publisher('/arduino/sensor_raw', String, queue_size=10)
        self.battery_pub = rospy.Publisher('/arduino/battery', Float32, queue_size=10)
        self.encoder_left_pub = rospy.Publisher('/arduino/encoder_left', Int32, queue_size=10)
        self.encoder_right_pub = rospy.Publisher('/arduino/encoder_right', Int32, queue_size=10)
        self.status_pub = rospy.Publisher('/arduino/status', String, queue_size=10)

        # Additional visualization publishers
        self.environment_marker_pub = rospy.Publisher('/environment_markers', MarkerArray, queue_size=10)
        self.sensor_markers_pub = rospy.Publisher('/sensor_markers', MarkerArray, queue_size=10)

        # Subscribers (ROS -> Arduino)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.led_sub = rospy.Subscriber('/arduino/led_control', Bool, self.led_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Mock hardware state
        self.motor_speeds = {'left': 0.0, 'right': 0.0}
        self.led_state = False
        self.encoder_counts = {'left': 0, 'right': 0}
        self.battery_voltage = 12.0

        # Robot position for environment awareness
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Simulation parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_separation = 0.25  # meters
        self.encoder_ticks_per_rev = 360
        self.publish_rate = rospy.Rate(20)  # 20 Hz

        # Environment parameters
        self.wall_x = 3.0
        
        # Start simulation thread
        self.simulation_thread = threading.Thread(target=self.simulate_hardware)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

        rospy.loginfo("Rosserial Mock Node initialized with environment visualization")

    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel speeds"""
        # Differential drive kinematics
        v = msg.linear.x   # Linear velocity
        w = msg.angular.z  # Angular velocity

        # Calculate wheel speeds
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        self.motor_speeds['left'] = v_left
        self.motor_speeds['right'] = v_right

        rospy.logdebug(f"Motor speeds: L={v_left:.2f}, R={v_right:.2f}")

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

    def led_callback(self, msg):
        """Control LED state"""
        self.led_state = msg.data
        rospy.logdebug(f"LED state: {self.led_state}")

    def simulate_hardware(self):
        """Simulate Arduino hardware in separate thread"""
        dt = 1.0 / 20.0  # 20 Hz simulation
        while not rospy.is_shutdown():
            # Simulate encoder counts
            # Convert wheel speeds to encoder increments
            left_rps = self.motor_speeds['left'] / (2 * 3.14159 * self.wheel_radius)  # Revolutions per second
            right_rps = self.motor_speeds['right'] / (2 * 3.14159 * self.wheel_radius)

            left_ticks = int(left_rps * self.encoder_ticks_per_rev * dt)
            right_ticks = int(right_rps * self.encoder_ticks_per_rev * dt)

            self.encoder_counts['left'] += left_ticks
            self.encoder_counts['right'] += right_ticks

            # Simulate battery discharge
            power_consumption = abs(self.motor_speeds['left']) + abs(self.motor_speeds['right'])
            self.battery_voltage -= power_consumption * 0.001 * dt
            self.battery_voltage = max(10.0, self.battery_voltage)

            # Add some noise
            self.battery_voltage += random.uniform(-0.01, 0.01)

            time.sleep(dt)

    def publish_arduino_data(self):
        """Publish simulated Arduino sensor data"""
        # Publish raw sensor data as string (mimicking Arduino serial output)
        front_dist = random.uniform(0.5, 3.0)
        left_dist = random.uniform(0.3, 2.0)  
        right_dist = random.uniform(0.3, 2.0)
        
        sensor_data = f"SENSORS,{front_dist:.2f},{left_dist:.2f},{right_dist:.2f}"
        self.sensor_pub.publish(String(data=sensor_data))

        # Publish battery voltage
        self.battery_pub.publish(Float32(data=self.battery_voltage))

        # Publish encoder counts
        self.encoder_left_pub.publish(Int32(data=self.encoder_counts['left']))
        self.encoder_right_pub.publish(Int32(data=self.encoder_counts['right']))

        # Publish status
        status = f"Motors: L={self.motor_speeds['left']:.2f}, R={self.motor_speeds['right']:.2f}, LED: {self.led_state}, Battery: {self.battery_voltage:.1f}V"
        self.status_pub.publish(String(data=status))

    def publish_environment_visualization(self):
        """Publish environment markers for walls and obstacles"""
        current_time = rospy.Time.now()
        marker_array = MarkerArray()

        # Ground plane marker
        ground_marker = Marker()
        ground_marker.header.frame_id = "odom"
        ground_marker.header.stamp = current_time
        ground_marker.ns = "environment"
        ground_marker.id = 0
        ground_marker.type = Marker.CUBE
        ground_marker.action = Marker.ADD
        ground_marker.pose.position.x = 0.0
        ground_marker.pose.position.y = 0.0
        ground_marker.pose.position.z = -0.01
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 10.0
        ground_marker.scale.y = 10.0
        ground_marker.scale.z = 0.02
        ground_marker.color.r = 0.9
        ground_marker.color.g = 0.9
        ground_marker.color.b = 0.9
        ground_marker.color.a = 0.5
        ground_marker.lifetime = rospy.Duration(0)

        # Wall marker (vertical at x=3.0)
        wall_marker = Marker()
        wall_marker.header.frame_id = "odom"
        wall_marker.header.stamp = current_time
        wall_marker.ns = "environment"
        wall_marker.id = 1
        wall_marker.type = Marker.CUBE
        wall_marker.action = Marker.ADD
        wall_marker.pose.position.x = self.wall_x
        wall_marker.pose.position.y = 0.0
        wall_marker.pose.position.z = 1.0
        wall_marker.pose.orientation.w = 1.0
        wall_marker.scale.x = 0.1  # 10cm thick
        wall_marker.scale.y = 8.0  # 8m long
        wall_marker.scale.z = 2.0  # 2m high
        wall_marker.color.r = 0.6
        wall_marker.color.g = 0.3
        wall_marker.color.b = 0.1
        wall_marker.color.a = 0.8
        wall_marker.lifetime = rospy.Duration(0)

        # Moving obstacle
        obstacle_x = 1.5 + 0.6 * math.sin(current_time.to_sec() * 0.3)
        obstacle_y = 0.5 + 0.4 * math.cos(current_time.to_sec() * 0.2)
        
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "odom"
        obstacle_marker.header.stamp = current_time
        obstacle_marker.ns = "environment"
        obstacle_marker.id = 2
        obstacle_marker.type = Marker.CYLINDER
        obstacle_marker.action = Marker.ADD
        obstacle_marker.pose.position.x = obstacle_x
        obstacle_marker.pose.position.y = obstacle_y
        obstacle_marker.pose.position.z = 0.5
        obstacle_marker.pose.orientation.w = 1.0
        obstacle_marker.scale.x = 0.3  # 30cm diameter
        obstacle_marker.scale.y = 0.3
        obstacle_marker.scale.z = 1.0
        obstacle_marker.color.r = 1.0
        obstacle_marker.color.g = 0.2
        obstacle_marker.color.b = 0.2
        obstacle_marker.color.a = 0.9
        obstacle_marker.lifetime = rospy.Duration(0)

        # Existing static obstacle marker
        static_obstacle_marker = Marker()
        static_obstacle_marker.header.frame_id = "odom"
        static_obstacle_marker.header.stamp = current_time
        static_obstacle_marker.ns = "environment"
        static_obstacle_marker.id = 3
        static_obstacle_marker.type = Marker.CUBE
        static_obstacle_marker.action = Marker.ADD
        static_obstacle_marker.pose.position.x = 2.0
        static_obstacle_marker.pose.position.y = -1.0
        static_obstacle_marker.pose.position.z = 0.25
        static_obstacle_marker.pose.orientation.w = 1.0
        static_obstacle_marker.scale.x = 0.5
        static_obstacle_marker.scale.y = 0.5
        static_obstacle_marker.scale.z = 0.5
        static_obstacle_marker.color.r = 0.3
        static_obstacle_marker.color.g = 0.3
        static_obstacle_marker.color.b = 0.8
        static_obstacle_marker.color.a = 0.8
        static_obstacle_marker.lifetime = rospy.Duration(0)

        # Four additional static obstacles with their own IDs
        


        marker_array.markers = [ground_marker, wall_marker, obstacle_marker, static_obstacle_marker]
        additional_obstacles = [
            {'id': 4, 'x': 1.0, 'y': 1.5, 'color': (0.1, 0.7, 0.1)},
            {'id': 5, 'x': -1.2, 'y': 2.0, 'color': (0.7, 0.1, 0.1)},
            {'id': 6, 'x': 0.5, 'y': -2.3, 'color': (0.1, 0.1, 0.7)},
            {'id': 7, 'x': -2.0, 'y': -1.0, 'color': (0.7, 0.7, 0.1)}
        ]

        for obs in additional_obstacles:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = current_time
            marker.ns = "environment"
            marker.id = obs['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = obs['color'][0]
            marker.color.g = obs['color'][1]
            marker.color.b = obs['color'][2]
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self.environment_marker_pub.publish(marker_array)

    def publish_sensor_visualization(self):
        """Publish sensor position markers"""
        current_time = rospy.Time.now()
        sensor_markers = MarkerArray()

        # Sensor positions relative to base_link
        sensors = [
            {"name": "front", "x": 0.15, "y": 0.0, "color": [0, 1, 0]},
            {"name": "left", "x": 0.0, "y": 0.1, "color": [0, 0, 1]},
            {"name": "right", "x": 0.0, "y": -0.1, "color": [1, 1, 0]}
        ]

        for i, sensor in enumerate(sensors):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = current_time
            marker.ns = "sensors"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = sensor["x"]
            marker.pose.position.y = sensor["y"] 
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.r = sensor["color"][0]
            marker.color.g = sensor["color"][1]
            marker.color.b = sensor["color"][2]
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            sensor_markers.markers.append(marker)

        self.sensor_markers_pub.publish(sensor_markers)

    def run(self):
        """Main publishing loop"""
        while not rospy.is_shutdown():
            self.publish_arduino_data()
            self.publish_environment_visualization()
            self.publish_sensor_visualization()
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        mock_node = RosserialMock()
        mock_node.run()
    except rospy.ROSInterruptException:
        pass