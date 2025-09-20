# Mobile Robot Navigation System
**Assignment 2 - Embedded Systems Design Course**  
**Manara University - م. باهر خير بك**

## Overview

This project implements a complete mobile robot navigation system using ROS (Robot Operating System) Noetic. The robot features differential drive locomotion, ultrasonic sensing for obstacle detection, and multiple navigation behaviors including obstacle avoidance, wall following, and goal-seeking.

## System Architecture

### Components

1. **Robot Model (URDF/Xacro)**
   - Differential drive mobile robot
   - Ultrasonic sensor for range detection
   - Proper kinematic chain with TF frames
   - Gazebo simulation support

2. **ROS Nodes**
   - `sensor_publisher`: Simulates/publishes ultrasonic sensor data
   - `robot_controller`: Main navigation and control logic
   - `mock_arduino`: Simulates Arduino hardware via rosserial

3. **Arduino Integration**
   - Real hardware support via rosserial
   - Ultrasonic sensor (HC-SR04) driver
   - Motor control (L298N driver) with differential drive kinematics
   - Battery monitoring and status reporting

4. **Navigation Behaviors**
   - **Obstacle Avoidance**: Basic reactive navigation
   - **Wall Following**: PID-controlled wall tracking
   - **Goto Position**: Point-to-point navigation with obstacle avoidance

## File Structure

```
mobile_robot_nav/
├── src/
│   ├── sensor_publisher.py      # Sensor simulation node
│   ├── robot_controller.py      # Main control logic
│   └── mock_arduino.py          # Arduino simulation
├── launch/
│   └── robot_system.launch      # Main system launcher
├── urdf/
│   ├── mobile_robot.urdf.xacro  # Robot model definition
│   └── mobile_robot.gazebo.xacro # Gazebo-specific configuration
├── config/
│   └── robot_params.yaml        # System parameters
├── arduino/
│   ├── ultrasonic_sensor.ino    # Arduino sensor code
│   └── motor_control.ino        # Arduino motor control
├── srv/
│   ├── ResetRobot.srv          # Robot reset service
│   └── ChangeBehavior.srv      # Behavior change service
├── package.xml
├── CMakeLists.txt
├── setup.py
└── README.md
```

## Prerequisites

### Software Requirements
- Ubuntu 20.04 LTS
- ROS Noetic (full desktop installation)
- Python 3.8+
- Arduino IDE (for hardware integration)

### Required ROS Packages
```bash
sudo apt install ros-noetic-rosserial-python
sudo apt install ros-noetic-rosserial-arduino 
sudo apt install ros-noetic-joint-state-publisher
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-urdf
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-tf2-geometry-msgs
```

## Installation and Setup

### 1. Create Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

### 2. Clone/Copy Package
```bash
# Copy the mobile_robot_nav folder to ~/catkin_ws/src/
cp -r mobile_robot_nav ~/catkin_ws/src/
```

### 3. Build Package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Make Python Scripts Executable
```bash
chmod +x ~/catkin_ws/src/mobile_robot_nav/src/*.py
```

## Usage Instructions

### Basic Launch (Simulation Mode)
```bash
# Terminal 1: Launch the robot system
roslaunch mobile_robot_nav robot_system.launch

# The system will start with:
# - RViz visualization
# - Mock Arduino simulation
# - All navigation nodes
# - Default obstacle avoidance behavior
```

### Launch with Gazebo Simulation
```bash
roslaunch mobile_robot_nav robot_system.launch use_gazebo:=true
```

### Launch with Real Hardware
1. Connect Arduino via USB
2. Upload appropriate Arduino code
3. Adjust port in launch file if needed
4. Launch:
```bash
roslaunch mobile_robot_nav robot_system.launch use_mock_arduino:=false
```

## Robot Control and Services

### Change Robot Behavior
```bash
# Obstacle avoidance mode
rosservice call /mobile_robot/change_behavior "behavior_mode: 'obstacle_avoidance' target_position: {x: 0, y: 0, z: 0}"

# Wall following mode  
rosservice call /mobile_robot/change_behavior "behavior_mode: 'wall_following' target_position: {x: 0, y: 0, z: 0}"

# Go to position mode
rosservice call /mobile_robot/change_behavior "behavior_mode: 'goto_position' target_position: {x: 2.0, y: 1.5, z: 0}"
```

### Reset Robot
```bash
rosservice call /mobile_robot/reset_robot "{new_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

### Manual Control (Testing)
```bash
# Move forward
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# Turn right
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'

# Stop
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
```

## Monitoring and Debugging

### View Active Topics
```bash
rostopic list
```

### Monitor Sensor Data
```bash
rostopic echo /ultrasonic_sensor
```

### Monitor Robot Velocity Commands
```bash
rostopic echo /cmd_vel
```

### Monitor Robot Path
```bash
rostopic echo /robot_path
```

### View TF Tree
```bash
rosrun tf2_tools view_frames.py
```

### Check Node Status
```bash
rosnode list
rosnode info /mobile_robot/robot_controller
```

## Hardware Setup (Optional)

### Arduino Connections

#### Ultrasonic Sensor (HC-SR04)
- VCC → 5V
- GND → GND  
- Trig → Pin 7
- Echo → Pin 8

#### Motor Driver (L298N)
- ENA → Pin 5 (PWM)
- IN1 → Pin 4
- IN2 → Pin 2
- ENB → Pin 6 (PWM)
- IN3 → Pin 7
- IN4 → Pin 8

#### Additional Components
- LED → Pin 13 (with 220Ω resistor)
- Battery monitor → Pin A0 (voltage divider)

### Arduino Libraries Required
- ros_lib (generated by rosserial)
- Standard Arduino libraries

## RViz Configuration

The system includes pre-configured RViz settings showing:
- Robot model (URDF visualization)
- TF frames and coordinate systems
- Ultrasonic sensor data visualization
- Robot path tracking
- Goal markers (in goto_position mode)
- Odometry information

## Parameters and Tuning

### Navigation Parameters (robot_params.yaml)
- `obstacle_threshold`: Distance threshold for obstacle detection (0.3m)
- `wall_following_distance`: Target distance for wall following (0.25m)
- `linear_velocity`: Maximum forward speed (0.2 m/s)
- `angular_velocity`: Maximum turning speed (0.5 rad/s)
- `goal_tolerance`: Distance tolerance for goal reaching (0.1m)

### PID Controller (Wall Following)
- `kp`: Proportional gain (1.0)
- `ki`: Integral gain (0.0)  
- `kd`: Derivative gain (0.1)

## Troubleshooting

### Common Issues

1. **"Package not found" error**
   - Ensure package is in catkin workspace
   - Run `catkin_make` and `source devel/setup.bash`

2. **Permission denied on Python scripts**
   - Run `chmod +x src/*.py` in package directory

3. **Arduino connection issues**
   - Check USB port (`ls /dev/ttyUSB*` or `/dev/ttyACM*`)
   - Verify baud rate matches (57600)
   - Ensure rosserial_python is installed

4. **RViz not showing robot model**
   - Verify robot_description parameter is loaded
   - Check URDF syntax with: `check_urdf robot.urdf`

5. **Robot not moving**
   - Check if cmd_vel topic is being published
   - Verify motor connections and power supply
   - Monitor Arduino serial output for errors

### Performance Tuning

- Adjust sensor update rate for smoother operation
- Tune PID parameters for better wall following
- Modify velocity limits based on hardware capabilities
- Adjust obstacle threshold based on environment

## Future Enhancements

- SLAM integration for mapping
- Advanced path planning algorithms
- Multi-sensor fusion
- Web-based control interface
- Machine learning for adaptive navigation
- Integration with ROS Navigation Stack

## Assignment Compliance Checklist

✅ **Robot Design**
- URDF/Xacro modeling with differential drive
- Ultrasonic sensor integration
- Proper joints and links structure

✅ **Simulation and Visualization**
- RViz visualization with TF frames
- Robot model display
- Sensor data visualization
- Path tracking visualization

✅ **ROS Nodes**
- Publisher node for sensor data simulation
- Subscriber node for sensor processing and control
- ROS services for behavior control and reset

✅ **Rosserial Integration**
- Mock Arduino node for simulation
- Real hardware Arduino code provided
- Serial communication setup

✅ **Documentation and Launch**
- Complete launch file system
- Comprehensive README
- Detailed system architecture documentation

## Author Information

**Student:** [Your Name]  
**Course:** Embedded Systems Design  
**Instructor:** م. باهر خير بك  
**University:** Manara University  
**Date:** September 2024  

## License

This project is developed for educational purposes as part of the Embedded Systems Design course at Manara University.
