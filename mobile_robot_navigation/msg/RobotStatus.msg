# RobotStatus.msg
# Robot status information

Header header
geometry_msgs/Pose2D current_pose    # Current robot position and orientation
geometry_msgs/Twist current_velocity # Current velocity
string behavior_mode                 # Current behavior (wall_follow, obstacle_avoid, go_to_goal)
bool emergency_stop                  # Emergency stop flag
float32 distance_traveled            # Total distance traveled
