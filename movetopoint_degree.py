import json
import time
import math
import FA
from simple_pid import PID  # Import PID controller
from PIDLineFollow import LineFollowPID_Fast, Loitering_indefinite
import os

# JSON file paths
ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
SLOW_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
OPPOSITE_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Fast.json"
TARGET_COORDS_Phase1 = "Automated-Overtaking/TargetCoordinates_Phase1.json"
TARGET_COORDS_Phase2 = "Automated-Overtaking/TargetCoordinates_Phase2.json"
TARGET_COORDS_Phase3 = "Automated-Overtaking/TargetCoordinates_Phase3.json"

# Function to read the latest robot or target position from a JSON file
def read_json(file_path):
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
            return data["robot"][0]  # Read the first entry
    except (FileNotFoundError, KeyError, json.JSONDecodeError):
        return None  # Return None if the file is missing or invalid

# Function to calculate the angle between two points
def calculate_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# Function to move the robot to phase 1 target position using PID control
def move_to_target_Phase1(fa_robot):
    
    base_speed = 20  # Base motor speed
    distance_tolerance = 30  # Distance within which the robot considers it has reached the target

    # Initialize PID controller for angle correction
    pid = PID(Kp=2.0, Ki=0.001, Kd=1.5, setpoint=0)  
    pid.output_limits = (-25, 25)  # Limit turn speed adjustment

    # Read the target coordinates outside the loop so they become fixed
    target_data = read_json(TARGET_COORDS_Phase1)
    if not target_data:
        print("Error: Could not read target JSON data.")
        target_data = read_json(TARGET_COORDS_Phase1)
        
    x_target = target_data["x"]
    y_target = target_data["y"]
    
    fixed_target_data = {"x": x_target, "y": y_target}
    with open('Automated-Overtaking/fixed_target_data_phase1.json', 'w') as f:
        json.dump(fixed_target_data, f)
        
    while True:
        current_loop_start = time.time() 
        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Calculate the target angle and error
        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]

        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance:
            print("Target reached!")
            fa_robot.SetMotors(0, 0)  # Stop motors
            with open('Automated-Overtaking/phase1_complete.json', 'w') as f:
                json.dump({"finished": True}, f)
                
            break

        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle error is less than 15 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed + 5
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 2
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        current_loop_end = time.time()
        loop_duration = current_loop_end - current_loop_start
        print(f"PID Loop Duration: {loop_duration:.6f} seconds")
        
        # print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")

# Function to move the robot to phase 2 target position using PID control
def move_to_target_Phase2(fa_robot):
    base_speed = 25  # Base forward speed
    distance_tolerance = 50  # Distance within which the robot considers it has reached the target
    timeout_duration = 10  # Maximum time allowed for the function to run in seconds
    start_time = time.time()  # Record the start time
    
    # Initialize PID controller for angle correction
    pid = PID(Kp=1.2, Ki=0.001, Kd=0.6, setpoint=0)
    pid.output_limits = (-50, 50)  # Limit turn speed adjustment

    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time > timeout_duration:
            print(f"Timeout reached after {timeout_duration} seconds. Transitioning to PullBack.")
            fa_robot.SetMotors(0, 0)  # Stop motors
            PullBack(fa_robot)
            Overtake_Check = False # 
            break # Exit the while loop and the function
        
        # Read the latest target coordinates
        target_data = read_json(TARGET_COORDS_Phase2)
        if not target_data:
            print("Error: Could not read target JSON data.")
            continue  # Retry reading

        x_target = target_data["x"]
        y_target = target_data["y"]

        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Calculate the target angle and error
        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]

        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance:
            print("Target reached!")
            fa_robot.SetMotors(0, 0)  # Stop motors
            break

        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 10 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed+10  # adjust for motor imbalance
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 2
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")

# Function to move the robot to phase 3 target position using PID control
def move_to_target_Phase3(fa_robot):
    base_speed = 25  # Base forward speed
    distance_tolerance = 50  # Distance within which the robot considers it has reached the target

    # Initialize PID controller for angle correction
    pid = PID(Kp=2.0, Ki=0.001, Kd=1.5, setpoint=0)  
    pid.output_limits = (-30, 30)  # Limit turn speed adjustment

    # Read the target coordinates once
    target_data = read_json(TARGET_COORDS_Phase3)
    if not target_data:
        print("Error: Could not read target JSON data.")
        target_data = read_json(TARGET_COORDS_Phase3)

    x_target = target_data["x"]
    y_target = target_data["y"]

    fixed_target_data = {"x": x_target, "y": y_target}
    with open('Automated-Overtaking/fixed_target_data_phase3.json', 'w') as f:
        json.dump(fixed_target_data, f)

    while True:
        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"] + 50
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Calculate the target angle and error
        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]

        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance:
            print("Target reached!")
            fa_robot.SetMotors(0, 0)  # Stop motors
            break

        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 15 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 2
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")
    
# Function to move the robot to phase 3.5 target position using PID control  
def move_to_target_Phase3_5(fa_robot):
    base_speed = 20  # Base forward speed
    distance_tolerance = 30  # Distance within which the robot considers it has reached the target

    # Initialize PID controller for angle correction
    pid = PID(Kp=2.0, Ki=0.001, Kd=1.5, setpoint=0) 
    pid.output_limits = (-30, 30)  # Limit turn speed adjustment

    # Read the target coordinates once
    target_data = read_json(TARGET_COORDS_Phase3)
    if not target_data:
        print("Error: Could not read target JSON data.")
        target_data = read_json(TARGET_COORDS_Phase3)
        

    x_target = target_data["x"] +150
    y_target = target_data["y"] 

    while True:
        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Calculate the target angle and error
        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]

        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance:
            print("Target reached!")
            fa_robot.SetMotors(0, 0)  # Stop motors
            break

        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 10 degrees
        if abs(angle_error) < 10:
            left_speed = base_speed+5
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 2
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")
  
# Function to pull back the robot after overtaking
def PullBack(fa_robot):
    base_speed = 25  # Base forward speed
    distance_tolerance = 50  # Distance within which the robot considers it has reached the target
    Pullback_Offset = 350  # Offset for the slow robot
    
    # Initialize PID controller for angle correction
    pid = PID(Kp=1.2, Ki=0.001, Kd=0.6, setpoint=0) 
    pid.output_limits = (-30, 30)  # Limit turn speed adjustment

    print("PullBack function initiated - Waiting for slow robot to pass overtake robot.")
    
    while True:
        # Read the latest target coordinates
        target_data = read_json(SLOW_ROBOT_COORDS_FILE)
        if not target_data:
            print("Error: Could not read target JSON data.")
            continue
        
        x_slow_robot = target_data["x"]
        y_slow_robot = target_data["y"]
        
        x_target = target_data["x"] - Pullback_Offset
        y_target = target_data["y"]
        
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        
        if x_robot < x_target:
            print("Slow robot has passed overtake robot in x-coordinate. Initiating pullback movement.")
            break # Exit the waiting loop
        else:
            print(f"Waiting - Fast Robot X: {x_robot}, Slow Robot X: {x_target}")
            time.sleep(0.2) # Wait a bit before checking again
    
    while True:
        # Read the latest target coordinates
        target_data = read_json(SLOW_ROBOT_COORDS_FILE)
        if not target_data:
            target_data = read_json(SLOW_ROBOT_COORDS_FILE)
            print("Error: Could not read target JSON data.")
            continue # Retry reading
          
        x_target = target_data["x"] - Pullback_Offset
        y_target = target_data["y"]

        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]
        print(f"Robot Position: ({x_robot}, {y_robot}), Angle: {angle_robot}")
        
        # Calculate the target angle and error
        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]

        print(x_target)
        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance:
            print("Target reached!")
            fa_robot.SetMotors(0, 0)  # Stop motors
            Loitering_indefinite(fa_robot)  # Call Loitering function
            break

        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 10 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed + 10
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 2
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)


def Overtake_P2P(robot):
    Overtake_Check = True
    move_to_target_Phase1(robot)
    move_to_target_Phase2(robot)
    if Overtake_Check == True:
        move_to_target_Phase3(robot)  
        move_to_target_Phase3_5(robot) 
        LineFollowPID_Fast(robot)
    

if __name__ == "__main__":
    # Initialize robot
    FA1 = FA.Create()
    FA1.ComOpen(6)

    move_to_target_Phase1(FA1)
    
    # Overtake_P2P(FA1)