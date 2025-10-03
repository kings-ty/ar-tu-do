#!/usr/bin/env python3

import os
import subprocess
import sys

def run_command(cmd, description):
    print(f"\n=== {description} ===")
    print(f"Running: {cmd}")
    
    if "gazebo" in cmd:
        # Run gazebo in background
        process = subprocess.Popen(cmd, shell=True)
        print(f"Started gazebo (PID: {process.pid})")
        return process
    else:
        # Run other commands and wait
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error: {result.stderr}")
        else:
            print("Success!")
        return result

def main():
    # Change to workspace directory
    workspace = "/home/ty/f1test_ws/ar-tu-do"
    os.chdir(workspace)
    
    # Source workspace
    source_cmd = "source install/setup.bash"
    
    print("=== Starting Car Simulation ===")
    
    # 1. Start Gazebo
    gazebo_cmd = f"{source_cmd} && ros2 launch gazebo_ros gazebo.launch.py world:={workspace}/install/racer_world/share/racer_world/worlds/racetrack_decorated.world"
    gazebo_process = run_command(gazebo_cmd, "Starting Gazebo")
    
    input("\nPress Enter when Gazebo is fully loaded...")
    
    # 2. Start robot_state_publisher
    urdf_path = f"{workspace}/ros_ws/src/simulation/racer_description/urdf/racer.xacro"
    robot_state_cmd = f'{source_cmd} && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro {urdf_path} simulation:=true)"'
    
    print("\nStarting robot_state_publisher in background...")
    robot_process = subprocess.Popen(robot_state_cmd, shell=True)
    
    input("\nPress Enter to spawn the car...")
    
    # 3. Spawn car
    spawn_cmd = f"{source_cmd} && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity racer_robot"
    run_command(spawn_cmd, "Spawning Car")
    
    input("\nPress Enter to test car movement...")
    
    # 4. Test movement
    test_cmd = f'{source_cmd} && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: 1.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"'
    run_command(test_cmd, "Testing Car Movement")
    
    print("\n=== Setup Complete! ===")
    print("You can now control the car with:")
    print("ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"")

if __name__ == "__main__":
    main()