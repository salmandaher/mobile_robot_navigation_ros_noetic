# SensorData.msg
# Custom message for sensor readings

Header header
float32 range_front      # Front ultrasonic sensor reading (meters)
float32 range_left       # Left ultrasonic sensor reading (meters) 
float32 range_right      # Right ultrasonic sensor reading (meters)
float32 battery_voltage  # Battery voltage
bool obstacle_detected   # Boolean flag for obstacle detection
