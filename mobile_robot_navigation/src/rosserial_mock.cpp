#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <random>

class RosserialMock {
private:
    ros::NodeHandle nh_;

    // Publishers (Arduino -> ROS)
    ros::Publisher sensor_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher encoder_left_pub_;
    ros::Publisher encoder_right_pub_;
    ros::Publisher status_pub_;

    // Subscribers (ROS -> Arduino)
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber led_sub_;

    // Mock hardware state
    struct MotorSpeeds {
        double left;
        double right;
    } motor_speeds_;

    bool led_state_;
    struct EncoderCounts {
        int left;
        int right;
    } encoder_counts_;

    double battery_voltage_;

    // Simulation parameters
    double wheel_radius_;
    double wheel_separation_;
    int encoder_ticks_per_rev_;

    // Timer
    ros::Timer publish_timer_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> voltage_noise_;
    std::uniform_real_distribution<double> sensor_noise_;

public:
    RosserialMock() : gen_(rd_()) {
        // Initialize parameters
        wheel_radius_ = 0.065;  // meters
        wheel_separation_ = 0.25;  // meters
        encoder_ticks_per_rev_ = 360;
        battery_voltage_ = 12.0;
        led_state_ = false;

        motor_speeds_.left = 0.0;
        motor_speeds_.right = 0.0;
        encoder_counts_.left = 0;
        encoder_counts_.right = 0;

        // Initialize random distributions
        voltage_noise_ = std::uniform_real_distribution<double>(-0.01, 0.01);
        sensor_noise_ = std::uniform_real_distribution<double>(0.1, 4.0);

        // Publishers
        sensor_pub_ = nh_.advertise<std_msgs::String>("/arduino/sensor_raw", 10);
        battery_pub_ = nh_.advertise<std_msgs::Float32>("/arduino/battery", 10);
        encoder_left_pub_ = nh_.advertise<std_msgs::Int32>("/arduino/encoder_left", 10);
        encoder_right_pub_ = nh_.advertise<std_msgs::Int32>("/arduino/encoder_right", 10);
        status_pub_ = nh_.advertise<std_msgs::String>("/arduino/status", 10);

        // Subscribers
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RosserialMock::cmdVelCallback, this);
        led_sub_ = nh_.subscribe("/arduino/led_control", 10, &RosserialMock::ledCallback, this);

        // Timer for publishing (20 Hz)
        publish_timer_ = nh_.createTimer(ros::Duration(1.0/20.0), 
                                       &RosserialMock::publishArduinoData, this);

        ROS_INFO("Rosserial Mock C++ node initialized");
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Convert Twist message to wheel speeds
        double v = msg->linear.x;  // Linear velocity
        double w = msg->angular.z; // Angular velocity

        // Calculate wheel speeds using differential drive kinematics
        motor_speeds_.left = v - (w * wheel_separation_ / 2.0);
        motor_speeds_.right = v + (w * wheel_separation_ / 2.0);

        ROS_DEBUG("Motor speeds: L=%.2f, R=%.2f", motor_speeds_.left, motor_speeds_.right);
    }

    void ledCallback(const std_msgs::Bool::ConstPtr& msg) {
        led_state_ = msg->data;
        ROS_DEBUG("LED state: %s", led_state_ ? "ON" : "OFF");
    }

    void simulateHardware() {
        double dt = 1.0 / 20.0;  // 20 Hz simulation

        // Simulate encoder counts
        double left_rps = motor_speeds_.left / (2 * M_PI * wheel_radius_);
        double right_rps = motor_speeds_.right / (2 * M_PI * wheel_radius_);

        int left_ticks = static_cast<int>(left_rps * encoder_ticks_per_rev_ * dt);
        int right_ticks = static_cast<int>(right_rps * encoder_ticks_per_rev_ * dt);

        encoder_counts_.left += left_ticks;
        encoder_counts_.right += right_ticks;

        // Simulate battery discharge
        double power_consumption = std::abs(motor_speeds_.left) + std::abs(motor_speeds_.right);
        battery_voltage_ -= power_consumption * 0.001 * dt;
        battery_voltage_ = std::max(10.0, battery_voltage_);

        // Add noise
        battery_voltage_ += voltage_noise_(gen_);
    }

    void publishArduinoData(const ros::TimerEvent& event) {
        // Update hardware simulation
        simulateHardware();

        // Publish raw sensor data as string (mimicking Arduino serial output)
        std_msgs::String sensor_msg;
        char sensor_buffer[100];
        snprintf(sensor_buffer, sizeof(sensor_buffer), "SENSORS,%.2f,%.2f,%.2f", 
                sensor_noise_(gen_), sensor_noise_(gen_), sensor_noise_(gen_));
        sensor_msg.data = std::string(sensor_buffer);
        sensor_pub_.publish(sensor_msg);

        // Publish battery voltage
        std_msgs::Float32 battery_msg;
        battery_msg.data = battery_voltage_;
        battery_pub_.publish(battery_msg);

        // Publish encoder counts
        std_msgs::Int32 encoder_left_msg;
        encoder_left_msg.data = encoder_counts_.left;
        encoder_left_pub_.publish(encoder_left_msg);

        std_msgs::Int32 encoder_right_msg;
        encoder_right_msg.data = encoder_counts_.right;
        encoder_right_pub_.publish(encoder_right_msg);

        // Publish status
        std_msgs::String status_msg;
        char status_buffer[200];
        snprintf(status_buffer, sizeof(status_buffer), 
                "Motors: L=%.2f, R=%.2f, LED: %s, Battery: %.1fV",
                motor_speeds_.left, motor_speeds_.right, 
                led_state_ ? "ON" : "OFF", battery_voltage_);
        status_msg.data = std::string(status_buffer);
        status_pub_.publish(status_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosserial_mock");

    RosserialMock mock_node;

    ros::spin();

    return 0;
}
