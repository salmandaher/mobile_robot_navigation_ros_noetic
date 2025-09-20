#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher', anonymous=True)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscriber for cmd_vel to simulate odometry
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Odometry publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Environment visualization publisher
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # Robot pose (simulated)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity
        self.vx = 0.0
        self.vth = 0.0

        # Time tracking
        self.last_time = rospy.Time.now()

        # Publishing rate
        self.rate = rospy.Rate(50)  # 50 Hz

        # Environment parameters
        self.wall_x = 3.0
        
        rospy.loginfo("TF Publisher initialized with environment visualization")

    def cmd_vel_callback(self, msg):
        """Update velocity from cmd_vel"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_odometry(self):
        """Update robot pose based on velocity commands"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Simple integration
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y

        self.x = max(-5.0, min(5.0, self.x))
        self.y = max(-5.0, min(5.0, self.y))

        self.theta += delta_theta

        self.last_time = current_time

    def publish_transforms(self):
        """Publish TF transforms including robot and sensor frames"""
        current_time = rospy.Time.now()

        # odom -> base_footprint transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        # Convert theta to quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation.x = quat[0]
        odom_trans.transform.rotation.y = quat[1]
        odom_trans.transform.rotation.z = quat[2]
        odom_trans.transform.rotation.w = quat[3]

        # base_footprint -> base_link transform
        base_trans = TransformStamped()
        base_trans.header.stamp = current_time
        base_trans.header.frame_id = "base_footprint"
        base_trans.child_frame_id = "base_link"
        base_trans.transform.translation.x = 0.0
        base_trans.transform.translation.y = 0.0
        base_trans.transform.translation.z = 0.1  # 10cm above ground
        base_trans.transform.rotation.x = 0.0
        base_trans.transform.rotation.y = 0.0
        base_trans.transform.rotation.z = 0.0
        base_trans.transform.rotation.w = 1.0

        # Sensor frame transforms
        # Front sensor
        front_sensor_trans = TransformStamped()
        front_sensor_trans.header.stamp = current_time
        front_sensor_trans.header.frame_id = "base_link"
        front_sensor_trans.child_frame_id = "front_sensor_link"
        front_sensor_trans.transform.translation.x = 0.15  # 15cm in front
        front_sensor_trans.transform.translation.y = 0.0
        front_sensor_trans.transform.translation.z = 0.05
        front_sensor_trans.transform.rotation.x = 0.0
        front_sensor_trans.transform.rotation.y = 0.0
        front_sensor_trans.transform.rotation.z = 0.0
        front_sensor_trans.transform.rotation.w = 1.0

        # Left sensor
        left_sensor_trans = TransformStamped()
        left_sensor_trans.header.stamp = current_time
        left_sensor_trans.header.frame_id = "base_link"
        left_sensor_trans.child_frame_id = "left_sensor_link"
        left_sensor_trans.transform.translation.x = 0.0
        left_sensor_trans.transform.translation.y = 0.1  # 10cm to the left
        left_sensor_trans.transform.translation.z = 0.05
        # Rotate 90 degrees to point left
        left_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        left_sensor_trans.transform.rotation.x = left_quat[0]
        left_sensor_trans.transform.rotation.y = left_quat[1]
        left_sensor_trans.transform.rotation.z = left_quat[2]
        left_sensor_trans.transform.rotation.w = left_quat[3]

        # Right sensor
        right_sensor_trans = TransformStamped()
        right_sensor_trans.header.stamp = current_time
        right_sensor_trans.header.frame_id = "base_link"
        right_sensor_trans.child_frame_id = "right_sensor_link"
        right_sensor_trans.transform.translation.x = 0.0
        right_sensor_trans.transform.translation.y = -0.1  # 10cm to the right
        right_sensor_trans.transform.translation.z = 0.05
        # Rotate -90 degrees to point right
        right_quat = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2)
        right_sensor_trans.transform.rotation.x = right_quat[0]
        right_sensor_trans.transform.rotation.y = right_quat[1]
        right_sensor_trans.transform.rotation.z = right_quat[2]
        right_sensor_trans.transform.rotation.w = right_quat[3]

        # Broadcast all transforms
        transforms = [odom_trans, base_trans, front_sensor_trans, left_sensor_trans, right_sensor_trans]
        self.tf_broadcaster.sendTransform(transforms)

    def publish_odometry(self):
        """Publish odometry message"""
        current_time = rospy.Time.now()

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Publish odometry
        self.odom_pub.publish(odom)

    def publish_environment_markers(self):
        """Publish visualization markers for wall and obstacle"""
        current_time = rospy.Time.now()
        marker_array = MarkerArray()

        # Wall marker
        wall_marker = Marker()
        wall_marker.header.frame_id = "odom"
        wall_marker.header.stamp = current_time
        wall_marker.ns = "environment"
        wall_marker.id = 0
        wall_marker.type = Marker.CUBE
        wall_marker.action = Marker.ADD
        wall_marker.pose.position.x = self.wall_x
        wall_marker.pose.position.y = 0.0
        wall_marker.pose.position.z = 1.0  # 2m high wall
        wall_marker.pose.orientation.x = 0.0
        wall_marker.pose.orientation.y = 0.0
        wall_marker.pose.orientation.z = 0.0
        wall_marker.pose.orientation.w = 1.0
        wall_marker.scale.x = 0.1  # 10cm thick
        wall_marker.scale.y = 6.0  # 6m wide
        wall_marker.scale.z = 2.0  # 2m high
        wall_marker.color.r = 0.8
        wall_marker.color.g = 0.8
        wall_marker.color.b = 0.8
        wall_marker.color.a = 1.0
        wall_marker.lifetime = rospy.Duration(0)  # Permanent

        # Moving obstacle marker
        obstacle_x = 1.5 + 0.5 * math.sin(current_time.to_sec() * 0.3)
        obstacle_y = 0.5 + 0.3 * math.cos(current_time.to_sec() * 0.2)
        
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "odom"
        obstacle_marker.header.stamp = current_time
        obstacle_marker.ns = "environment"
        obstacle_marker.id = 1
        obstacle_marker.type = Marker.CYLINDER
        obstacle_marker.action = Marker.ADD
        obstacle_marker.pose.position.x = obstacle_x
        obstacle_marker.pose.position.y = obstacle_y
        obstacle_marker.pose.position.z = 0.5  # 1m high
        obstacle_marker.pose.orientation.x = 0.0
        obstacle_marker.pose.orientation.y = 0.0
        obstacle_marker.pose.orientation.z = 0.0
        obstacle_marker.pose.orientation.w = 1.0
        obstacle_marker.scale.x = 0.4  # 40cm diameter
        obstacle_marker.scale.y = 0.4
        obstacle_marker.scale.z = 1.0  # 1m high
        obstacle_marker.color.r = 1.0
        obstacle_marker.color.g = 0.0
        obstacle_marker.color.b = 0.0
        obstacle_marker.color.a = 1.0
        obstacle_marker.lifetime = rospy.Duration(0)

        # Robot representation marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "base_link"
        robot_marker.header.stamp = current_time
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = 0.0
        robot_marker.pose.position.y = 0.0
        robot_marker.pose.position.z = 0.05
        robot_marker.pose.orientation.x = 0.0
        robot_marker.pose.orientation.y = 0.0
        robot_marker.pose.orientation.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.3  # 30cm diameter
        robot_marker.scale.y = 0.3
        robot_marker.scale.z = 0.1  # 10cm high
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        robot_marker.color.a = 0.8
        robot_marker.lifetime = rospy.Duration(0)

        marker_array.markers = [wall_marker, obstacle_marker, robot_marker]
        self.marker_pub.publish(marker_array)

    def run(self):
        """Main publishing loop"""
        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_transforms()
            self.publish_odometry()
            self.publish_environment_markers()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tf_pub = TFPublisher()
        tf_pub.run()
    except rospy.ROSInterruptException:
        pass