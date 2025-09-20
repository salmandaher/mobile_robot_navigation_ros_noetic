#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <mobile_robot_navigation/SensorData.h>
#include <mobile_robot_navigation/RobotStatus.h>
#include <mobile_robot_navigation/ResetPosition.h>
#include <mobile_robot_navigation/ChangeBehavior.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class RobotController {
private:
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;

    // Subscribers
    ros::Subscriber sensor_sub_;
    ros::Subscriber odom_sub_;

    // Services
    ros::ServiceServer reset_service_;
    ros::ServiceServer behavior_service_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Robot state
    geometry_msgs::Pose2D current_pose_;
    geometry_msgs::Twist current_velocity_;
    mobile_robot_navigation::SensorData::ConstPtr sensor_data_;
    std::string behavior_mode_;
    geometry_msgs::Point target_point_;
    bool emergency_stop_;
    double distance_traveled_;

    // Control parameters
    double max_linear_speed_;
    double max_angular_speed_;
    double obstacle_threshold_;
    double wall_follow_distance_;
    double goal_tolerance_;

    // Timer
    ros::Timer control_timer_;

public:
    RobotController() : tf_listener_(tf_buffer_) {
        // Parameters
        ros::NodeHandle pnh("~");
        pnh.param("max_linear_speed", max_linear_speed_, 0.5);
        pnh.param("max_angular_speed", max_angular_speed_, 1.0);
        pnh.param("obstacle_threshold", obstacle_threshold_, 0.5);
        pnh.param("wall_follow_distance", wall_follow_distance_, 0.3);
        pnh.param("goal_tolerance", goal_tolerance_, 0.1);

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        status_pub_ = nh_.advertise<mobile_robot_navigation::RobotStatus>("/robot_status", 10);

        // Subscribers
        sensor_sub_ = nh_.subscribe("/sensor_data", 10, &RobotController::sensorCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 10, &RobotController::odomCallback, this);

        // Services
        reset_service_ = nh_.advertiseService("/reset_position", 
                                            &RobotController::resetPositionCallback, this);
        behavior_service_ = nh_.advertiseService("/change_behavior", 
                                               &RobotController::changeBehaviorCallback, this);

        // Initialize state
        behavior_mode_ = "obstacle_avoid";
        emergency_stop_ = false;
        distance_traveled_ = 0.0;

        // Control timer (10 Hz)
        control_timer_ = nh_.createTimer(ros::Duration(0.1), &RobotController::controlLoop, this);

        ROS_INFO("Robot Controller C++ node initialized");
    }

    void sensorCallback(const mobile_robot_navigation::SensorData::ConstPtr& msg) {
        sensor_data_ = msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Update current pose
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;

        current_velocity_ = msg->twist.twist;
    }

    geometry_msgs::Twist obstacleAvoidanceBehavior() {
        geometry_msgs::Twist cmd_vel;

        if (!sensor_data_) {
            return cmd_vel;
        }

        // Check for obstacles
        bool front_clear = sensor_data_->range_front > obstacle_threshold_;
        bool left_clear = sensor_data_->range_left > obstacle_threshold_;
        bool right_clear = sensor_data_->range_right > obstacle_threshold_;

        if (front_clear) {
            // Move forward
            cmd_vel.linear.x = max_linear_speed_ * 0.5;
            cmd_vel.angular.z = 0.0;
        } else {
            // Obstacle detected, turn away
            if (left_clear && !right_clear) {
                // Turn left
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = max_angular_speed_ * 0.7;
            } else if (right_clear && !left_clear) {
                // Turn right
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -max_angular_speed_ * 0.7;
            } else {
                // Turn towards the side with more space
                if (sensor_data_->range_left > sensor_data_->range_right) {
                    cmd_vel.angular.z = max_angular_speed_ * 0.7;
                } else {
                    cmd_vel.angular.z = -max_angular_speed_ * 0.7;
                }
                cmd_vel.linear.x = 0.0;
            }
        }

        return cmd_vel;
    }

    geometry_msgs::Twist wallFollowBehavior() {
        geometry_msgs::Twist cmd_vel;

        if (!sensor_data_) {
            return cmd_vel;
        }

        // PD control for wall following
        double error = sensor_data_->range_left - wall_follow_distance_;

        // Proportional control
        double kp = 2.0;
        double angular_correction = kp * error;

        // Limit angular velocity
        angular_correction = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_correction));

        // Check front obstacle
        if (sensor_data_->range_front > obstacle_threshold_) {
            cmd_vel.linear.x = max_linear_speed_ * 0.3;
            cmd_vel.angular.z = angular_correction;
        } else {
            // Turn right to avoid front obstacle
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -max_angular_speed_ * 0.5;
        }

        return cmd_vel;
    }

    bool resetPositionCallback(mobile_robot_navigation::ResetPosition::Request& req,
                              mobile_robot_navigation::ResetPosition::Response& res) {
        try {
            current_pose_ = req.new_pose;
            distance_traveled_ = 0.0;
            res.success = true;
            res.message = "Position reset successfully";
            ROS_INFO("Robot position reset to (%.2f, %.2f, %.2f)", 
                    req.new_pose.x, req.new_pose.y, req.new_pose.theta);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = std::string("Failed to reset position: ") + e.what();
            ROS_ERROR("%s", res.message.c_str());
        }

        return true;
    }

    bool changeBehaviorCallback(mobile_robot_navigation::ChangeBehavior::Request& req,
                               mobile_robot_navigation::ChangeBehavior::Response& res) {
        res.previous_mode = behavior_mode_;

        std::vector<std::string> valid_modes = {"obstacle_avoid", "wall_follow", "go_to_goal"};

        if (std::find(valid_modes.begin(), valid_modes.end(), req.behavior_mode) != valid_modes.end()) {
            behavior_mode_ = req.behavior_mode;
            if (req.behavior_mode == "go_to_goal") {
                target_point_ = req.target;
            }
            res.success = true;
            res.message = "Behavior changed to " + req.behavior_mode;
            ROS_INFO("%s", res.message.c_str());
        } else {
            res.success = false;
            res.message = "Invalid behavior mode";
            ROS_ERROR("%s", res.message.c_str());
        }

        return true;
    }

    void controlLoop(const ros::TimerEvent& event) {
        geometry_msgs::Twist cmd_vel;

        if (emergency_stop_) {
            // Stop the robot
            cmd_vel = geometry_msgs::Twist();
        } else {
            // Execute behavior based on current mode
            if (behavior_mode_ == "obstacle_avoid") {
                cmd_vel = obstacleAvoidanceBehavior();
            } else if (behavior_mode_ == "wall_follow") {
                cmd_vel = wallFollowBehavior();
            } else {
                cmd_vel = geometry_msgs::Twist();  // Default stop
            }
        }

        // Publish control commands
        cmd_vel_pub_.publish(cmd_vel);

        // Publish robot status
        publishStatus();
    }

    void publishStatus() {
        mobile_robot_navigation::RobotStatus status_msg;
        status_msg.header.stamp = ros::Time::now();
        status_msg.header.frame_id = "base_link";
        status_msg.current_pose = current_pose_;
        status_msg.current_velocity = current_velocity_;
        status_msg.behavior_mode = behavior_mode_;
        status_msg.emergency_stop = emergency_stop_;
        status_msg.distance_traveled = distance_traveled_;

        status_pub_.publish(status_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");

    RobotController controller;

    ros::spin();

    return 0;
}
