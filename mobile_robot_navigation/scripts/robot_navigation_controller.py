#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point, Pose2D
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mobile_robot_navigation.msg import SensorData, RobotStatus
from mobile_robot_navigation.srv import ResetPosition, ResetPositionResponse
from mobile_robot_navigation.srv import ChangeBehavior, ChangeBehaviorResponse
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PointStamped

class RobotNavigationController:
    def __init__(self):
        rospy.init_node('robot_navigation_controller', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)

        # Subscribers
        self.sensor_sub = rospy.Subscriber('/sensor_data', SensorData, self.sensor_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Services
        self.reset_service = rospy.Service('/reset_position', ResetPosition, self.reset_position_callback)
        self.behavior_service = rospy.Service('/change_behavior', ChangeBehavior, self.change_behavior_callback)

        # Robot state variables
        self.current_pose = Pose2D()
        self.current_velocity = Twist()
        self.sensor_data = None
        self.behavior_mode = "obstacle_avoid"  # Default behavior
        self.target_point = Point()
        self.emergency_stop = False
        self.distance_traveled = 0.0
        self.last_pose = Pose2D()

        # Path tracking
        self.robot_path = Path()
        self.robot_path.header.frame_id = "odom"

        # Control parameters - Enhanced for realistic environment
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)  # Reduced for safety
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.6)  # More conservative
        self.wall_follow_distance = rospy.get_param('~wall_follow_distance', 0.4)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)

        # Environment awareness
        self.wall_x = 3.0  # Known wall position
        self.safe_distance = 0.3  # Minimum safe distance from obstacles

        # Control loop
        self.control_rate = rospy.Rate(20)  # Increased to 20 Hz for better responsiveness

        rospy.loginfo("Robot Navigation Controller initialized - Enhanced environment awareness")

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.sensor_data = msg
        
        # Emergency stop if very close obstacle
        if msg.range_front < 0.2 or msg.range_left < 0.15 or msg.range_right < 0.15:
            self.emergency_stop = True
            rospy.logwarn("Emergency stop triggered - obstacle too close!")
        else:
            self.emergency_stop = False

    def odom_callback(self, msg):
        """Process odometry data"""
        # Update current pose
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        import tf.transformations
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_pose.theta = yaw

        self.current_velocity = msg.twist.twist

        # Update distance traveled
        if hasattr(self, 'last_pose'):
            dx = self.current_pose.x - self.last_pose.x
            dy = self.current_pose.y - self.last_pose.y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)

        self.last_pose = Pose2D(x=self.current_pose.x, y=self.current_pose.y, theta=self.current_pose.theta)

        # Add to path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose = msg.pose.pose
        self.robot_path.poses.append(pose_stamped)

        # Limit path length
        if len(self.robot_path.poses) > 500:
            self.robot_path.poses = self.robot_path.poses[-500:]

    def enhanced_obstacle_avoidance_behavior(self):
        """Enhanced obstacle avoidance with environment awareness"""
        if not self.sensor_data:
            return Twist()

        cmd_vel = Twist()

        # Get sensor readings
        front_range = self.sensor_data.range_front
        left_range = self.sensor_data.range_left  
        right_range = self.sensor_data.range_right

        # Check clearance in each direction
        front_clear = front_range > self.obstacle_threshold
        left_clear = left_range > self.safe_distance
        right_clear = right_range > self.safe_distance

        # Enhanced decision making
        if front_clear:
            # Path ahead is clear, move forward with adaptive speed
            speed_factor = min(1.0, (front_range - self.safe_distance) / self.obstacle_threshold)
            cmd_vel.linear.x = self.max_linear_speed * speed_factor * 0.7
            
            # Minor corrections based on side sensors
            if left_range < right_range and left_range < 1.0:
                cmd_vel.angular.z = -0.2  # Turn slightly right
            elif right_range < left_range and right_range < 1.0:
                cmd_vel.angular.z = 0.2   # Turn slightly left

        else:
            # Obstacle ahead - need to turn
            cmd_vel.linear.x = 0.0
            
            if left_clear and right_clear:
                # Both sides clear - turn towards more open space
                if left_range > right_range:
                    cmd_vel.angular.z = self.max_angular_speed * 0.6
                else:
                    cmd_vel.angular.z = -self.max_angular_speed * 0.6
            elif left_clear:
                # Turn left
                cmd_vel.angular.z = self.max_angular_speed * 0.7
            elif right_clear:
                # Turn right
                cmd_vel.angular.z = -self.max_angular_speed * 0.7
            else:
                # Both sides blocked - reverse turn
                cmd_vel.linear.x = -0.1
                cmd_vel.angular.z = self.max_angular_speed

        rospy.logdebug(f"Obstacle avoidance: F={front_range:.2f}, L={left_range:.2f}, R={right_range:.2f}")
        return cmd_vel

    def wall_follow_behavior(self):
        """Enhanced wall following behavior"""
        if not self.sensor_data:
            return Twist()

        cmd_vel = Twist()
        
        # PD control for wall following using left sensor
        error = self.sensor_data.range_left - self.wall_follow_distance
        
        # Proportional control with saturation
        kp = 1.5
        angular_correction = kp * error
        angular_correction = max(-self.max_angular_speed*0.5, 
                                min(self.max_angular_speed*0.5, angular_correction))

        # Check front obstacle
        if self.sensor_data.range_front > self.obstacle_threshold:
            # Continue forward with wall following
            cmd_vel.linear.x = self.max_linear_speed * 0.4
            cmd_vel.angular.z = angular_correction
        else:
            # Turn away from front obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -self.max_angular_speed * 0.6

        return cmd_vel

    def go_to_goal_behavior(self):
        """Navigate to target with obstacle avoidance"""
        cmd_vel = Twist()

        # Calculate distance and angle to goal
        dx = self.target_point.x - self.current_pose.x
        dy = self.target_point.y - self.current_pose.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_pose.theta

        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi

        if distance_to_goal > self.goal_tolerance:
            # First check for obstacles
            if self.sensor_data and self.sensor_data.range_front < self.obstacle_threshold:
                # Obstacle detected - use obstacle avoidance
                return self.enhanced_obstacle_avoidance_behavior()
            
            # Control angular velocity to face the goal
            if abs(angle_error) > 0.2:
                cmd_vel.angular.z = 1.5 * angle_error
                cmd_vel.linear.x = self.max_linear_speed * 0.2  # Slow while turning
            else:
                # Move towards goal
                cmd_vel.linear.x = min(self.max_linear_speed * 0.6, distance_to_goal)
                cmd_vel.angular.z = 0.8 * angle_error
        else:
            # Goal reached
            rospy.loginfo("Goal reached!")
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def exploration_behavior(self):
        """Simple exploration behavior"""
        if not self.sensor_data:
            return Twist()
            
        # Similar to obstacle avoidance but with more forward bias
        cmd_vel = self.enhanced_obstacle_avoidance_behavior()
        
        # Add slight random component for exploration
        if abs(cmd_vel.angular.z) < 0.1 and cmd_vel.linear.x > 0:
            import random
            cmd_vel.angular.z += random.uniform(-0.2, 0.2)
            
        return cmd_vel

    def reset_position_callback(self, req):
        """Service callback to reset robot position"""
        response = ResetPositionResponse()
        try:
            self.current_pose = req.new_pose
            self.distance_traveled = 0.0
            # Clear path
            self.robot_path.poses = []
            response.success = True
            response.message = "Position reset successfully"
            rospy.loginfo(f"Robot position reset to ({req.new_pose.x}, {req.new_pose.y}, {req.new_pose.theta})")
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset position: {str(e)}"
            rospy.logerr(response.message)
        return response

    def change_behavior_callback(self, req):
        """Service callback to change robot behavior"""
        response = ChangeBehaviorResponse()
        response.previous_mode = self.behavior_mode

        valid_modes = ["obstacle_avoid", "wall_follow", "go_to_goal", "exploration"]

        if req.behavior_mode in valid_modes:
            self.behavior_mode = req.behavior_mode
            if req.behavior_mode == "go_to_goal":
                self.target_point = req.target
            response.success = True
            response.message = f"Behavior changed to {req.behavior_mode}"
            rospy.loginfo(response.message)
        else:
            response.success = False
            response.message = f"Invalid behavior mode. Valid modes: {valid_modes}"
            rospy.logerr(response.message)
        return response

    def publish_status(self):
        """Publish robot status"""
        status_msg = RobotStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.header.frame_id = "base_link"
        self.current_pose.x = max(-5.0, min(5.0, self.current_pose.x))
        self.current_pose.y = max(-5.0, min(5.0, self.current_pose.y))
        status_msg.current_pose = self.current_pose
        status_msg.current_velocity = self.current_velocity
        status_msg.behavior_mode = self.behavior_mode
        status_msg.emergency_stop = self.emergency_stop
        status_msg.distance_traveled = self.distance_traveled

        self.status_pub.publish(status_msg)

    def run(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            if self.emergency_stop:
                cmd_vel = Twist()  # Stop the robot
                rospy.logwarn_throttle(1.0, "Robot stopped - obstacle too close")
            else:
                # Execute behavior based on current mode
                if self.behavior_mode == "obstacle_avoid":
                    cmd_vel = self.enhanced_obstacle_avoidance_behavior()
                elif self.behavior_mode == "wall_follow":
                    cmd_vel = self.wall_follow_behavior()
                elif self.behavior_mode == "go_to_goal":
                    cmd_vel = self.go_to_goal_behavior()
                elif self.behavior_mode == "exploration":
                    cmd_vel = self.exploration_behavior()
                else:
                    cmd_vel = Twist()  # Default stop

            # Apply velocity limits
            cmd_vel.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd_vel.linear.x))
            cmd_vel.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd_vel.angular.z))
            

            # Publish control commands
            self.cmd_vel_pub.publish(cmd_vel)

            # Publish robot status
            self.publish_status()

            # Publish path
            self.robot_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.robot_path)

            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotNavigationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass