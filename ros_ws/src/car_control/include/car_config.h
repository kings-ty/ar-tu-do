#pragma once

namespace car_config {
    // Motor configuration
    constexpr double MAX_RPM_ELECTRICAL = 2000.0;  // Maximum RPM for the motor
    
    // Servo configuration  
    constexpr double MAX_SERVO_POSITION = 1.0;     // Maximum servo position (normalized)
    
    // Safety limits
    constexpr double MAX_SPEED = 5.0;              // Maximum speed in m/s
    constexpr double MAX_STEERING_ANGLE = 0.34;    // Maximum steering angle in radians (~20 degrees)
}