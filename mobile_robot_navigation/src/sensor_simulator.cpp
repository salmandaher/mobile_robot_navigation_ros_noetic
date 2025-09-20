#include <ros/ros.h>
#include <mobile_robot_navigation/SensorData.h>
#include <std_msgs/Header.h>
#include <random>
#include <cmath>

class SensorSimulator {
private:
    ros::NodeHandle nh_;
    ros::Publisher sensor_pub_;
    ros::Timer timer_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_dist_;

    // Parameters
    double max_range_;
    double min_range_;
    double noise_std_;
    double publish_rate_;

public:
    SensorSimulator() : gen_(rd_()) {
        // Get parameters
        ros::NodeHandle pnh("~");
        pnh.param("max_range", max_range_, 4.0);
        pnh.param("min_range", min_range_, 0.04);
        pnh.param("noise_std", noise_std_, 0.02);
        pnh.param("publish_rate", publish_rate_, 10.0);

        // Initialize noise distribution
        noise_dist_ = std::normal_distribution<double>(0.0, noise_std_);

        // Publisher
        sensor_pub_ = nh_.advertise<mobile_robot_navigation::SensorData>("/sensor_data", 10);

        // Timer for periodic publishing
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                &SensorSimulator::publishSensorData, this);

        ROS_INFO("Sensor Simulator C++ node initialized");
    }

    double addNoise(double value) {
        double noise = noise_dist_(gen_);
        return std::max(min_range_, std::min(max_range_, value + noise));
    }

    void simulateEnvironment(double& front_range, double& left_range, double& right_range) {
        double current_time = ros::Time::now().toSec();

        // Create varying obstacle distances over time
        double front_base = 2.0 + 0.5 * std::sin(current_time * 0.1);
        double left_base = 0.8 + 0.2 * std::sin(current_time * 0.15);
        double right_base = 3.0 + 0.3 * std::cos(current_time * 0.12);

        // Add noise to each reading
        front_range = addNoise(front_base);
        left_range = addNoise(left_base);
        right_range = addNoise(right_base);
    }

    void publishSensorData(const ros::TimerEvent& event) {
        mobile_robot_navigation::SensorData sensor_msg;

        // Set header
        sensor_msg.header.stamp = ros::Time::now();
        sensor_msg.header.frame_id = "ultrasonic_front";

        // Simulate sensor readings
        double front_range, left_range, right_range;
        simulateEnvironment(front_range, left_range, right_range);

        sensor_msg.range_front = front_range;
        sensor_msg.range_left = left_range;
        sensor_msg.range_right = right_range;
        sensor_msg.battery_voltage = 12.0 + (std::rand() % 100 - 50) / 100.0;
        sensor_msg.obstacle_detected = front_range < 0.5;

        sensor_pub_.publish(sensor_msg);

        ROS_DEBUG("Published sensor data: F=%.2f, L=%.2f, R=%.2f", 
                 front_range, left_range, right_range);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_simulator");

    SensorSimulator simulator;

    ros::spin();

    return 0;
}
