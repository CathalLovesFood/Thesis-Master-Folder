import serial
import time
import FA
import json
import math #test
from simple_pid import PID  # Import the PID library

ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
SLOW_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
OPPOSITE_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Fast.json"
TARGET_COORDS_Phase1 = "Automated-Overtaking/TargetCoordinates_Phase1.json"
TARGET_COORDS_Phase2 = "Automated-Overtaking/TargetCoordinates_Phase2.json"
TARGET_COORDS_Phase3 = "Automated-Overtaking/TargetCoordinates_Phase3.json"

# This function calculates the angle between two points
def calculate_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# This function reads the JSON file and returns the info the JSON file
def read_json(file_path):
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
            return data["robot"][0]  # Read the first entry
    except (FileNotFoundError, KeyError, json.JSONDecodeError):
        return None  # Return None if the file is missing or invalid

#  Line following function using PID control
def LineFollowPID_Fast(Robot):
    with open('Automated-Overtaking/OvertakeCoordinates_Fast.json', 'w') as f:
        json.dump({}, f)
    
    
    OVERTAKE_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
    SLOW_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
    OPPOSITE_COORDS_FILE = "Automated-Overtaking\OvertakeCoordinates_Fast.json"
    
    base_speed = 20        # Base speed for the robot
    base_speed_left = 30
    max_speed = 50         # Maximum allowed speed

    distance_tolerance = 300  # Distance within which the robot considers initiating Overtake

    # PID gains - You will need to tune these values
    Kp = 1    # Proportional gain
    Ki = 0.00 # Integral gain
    Kd = 0.06 # Derivative gain

    # Initialize PID controller
    pid = PID(Kp, Ki, Kd, setpoint=0) 
    pid.output_limits = (-max_speed, max_speed)  # Limit the output to the motor speed range

    last_time = time.time()

    while True:
        current_loop_start = time.time()    
    
        # Read sensor values
        leftSensor = Robot.ReadLine(0)
        rightSensor = Robot.ReadLine(1)

        # Calculate error
        error = rightSensor - leftSensor
        error = error / 6

        current_time = time.time()
        dt = current_time - last_time

        with open('dt_log.txt', 'a') as log_file:
            log_file.write(f"{dt}\n")
        
        # Calculate PID output
        pid_output = pid(error, dt=dt)

        # Adjust motor speeds based on PID output
        left_motor_speed =  base_speed - pid_output
        right_motor_speed = base_speed_left + pid_output

        # Clamp motor speeds to be within valid range
        left_motor_speed = max(0, min(max_speed, left_motor_speed))
        right_motor_speed = max(0, min(max_speed, right_motor_speed))

        # Set motor speeds
        Robot.SetMotors(int(right_motor_speed), int(left_motor_speed))

        current_loop_end = time.time()
        loop_duration = current_loop_end - current_loop_start
        print(f"PID Loop Duration: {loop_duration:.6f} seconds")

        last_error = error
        last_time = current_time

        # Update robot positions
        Overtake_robot_data = read_json(OVERTAKE_COORDS_FILE)
        if not Overtake_robot_data:
            # print("Error: Could not read Overtake JSON data.")
            continue  # Retry reading
        
        Slow_robot_data = read_json(SLOW_COORDS_FILE)
        if not Slow_robot_data:
            # print("Error: Could not read slow JSON data.")
            continue  # Retry reading
        
        Opposite_robot_data = read_json(OPPOSITE_COORDS_FILE)
        if not Opposite_robot_data:
            # print("Error: Could not read opposite JSON data.")
            continue
        
        x_overtake = Overtake_robot_data["x"]
        y_overtake = Overtake_robot_data["y"]

        x_slow = Slow_robot_data["x"]
        y_slow = Slow_robot_data["y"]

        # Calculate distance
        distance = abs(x_slow - x_overtake)
        print(f"Updated Distance During Line Follow: {distance}")
        
        if distance < distance_tolerance:
            print("Distance condition met")

            if Opposite_robot_data and all(k in Opposite_robot_data for k in ("x", "angle")):
                x_opposite = Opposite_robot_data["x"]
                angle_Opposite = Opposite_robot_data["angle"]

                if x_opposite > x_overtake and (angle_Opposite >= 160 or angle_Opposite <= -160):
                    print("Car in other lane initiating holding pattern.")
                    Loitering(Robot)
            
            return  # Exit the loop to transition to the overtake maneuver
        
        

# loitering function to hold the robot behind slower vehicle
def Loitering(fa_robot):
    
    # Clears the JSON file to ensure no old data is present 
    with open('Automated-Overtaking/OvertakeCoordinates_Fast.json', 'w') as f:
        json.dump({}, f)
    
    ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
    SLOW_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
    OPPOSITE_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Fast.json"
    
    base_speed = 25  # Base forward speed
    distance_tolerance = 50  # Distance within which the robot considers it has reached the target

    # Initialize PID controller for angle correction
    pid = PID(Kp=1.2, Ki=0.001, Kd=0.6, setpoint=0)  
    pid.output_limits = (-30, 30)  # Limit turn speed adjustment

    while True:
        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Read the target coordinates once
        target_data = read_json(SLOW_ROBOT_COORDS_FILE)
        if not target_data:
            print("Error: Could not read target JSON data.")
            target_data = read_json(SLOW_ROBOT_COORDS_FILE)

        x_R_target = target_data["x"]
        x_target = target_data["x"] - 350
        y_target = target_data["y"]

        Opposite_robot_data = read_json(OPPOSITE_ROBOT_COORDS_FILE)
        if not Opposite_robot_data:
            print("Error: Could not read opposite JSON data.")
            Opposite_robot_data = read_json(OPPOSITE_ROBOT_COORDS_FILE)
            continue  # Retry reading
        
        if Opposite_robot_data and all(k in Opposite_robot_data for k in ("x", "angle")):
            x_opposite = Opposite_robot_data["x"]
            angle_Opposite = Opposite_robot_data["angle"]

        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]
        
        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        distance_backwards = math.sqrt((x_R_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance or distance_backwards < 300:
            print("Target reached!")
            
            if x_opposite < x_robot-100 or x_opposite == 0:
                print("Safe to overtake.")
                break
            
            print("Not Safe to overtake. Road not clear.")
            fa_robot.SetMotors(0, 0)  # Stop motors
            time.sleep(1)
            
        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 10 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 1.5
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")

        # time.sleep(0.1)  # Small delay for control loop timing
       
# This function is called when the robot is loitering indefinitely behind a slower vehicle
def Loitering_indefinite(fa_robot):
    
    ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
    SLOW_ROBOT_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
    
    """Moves the robot to the target using a PID-based control for angle correction."""
    base_speed = 25  # Base forward speed
    distance_tolerance = 50  # Distance within which the robot considers it has reached the target

    # Initialize PID controller for angle correction
    pid = PID(Kp=1.2, Ki=0.001, Kd=0.6, setpoint=0)  # Tune these values as needed
    pid.output_limits = (-30, 30)  # Limit turn speed adjustment

    
    while True:
        # Read the latest robot position
        robot_data = read_json(ROBOT_COORDS_FILE)
        if not robot_data:
            print("Error: Could not read robot JSON data.")
            continue  # Retry reading

        x_robot = robot_data["x"]
        y_robot = robot_data["y"]
        angle_robot = robot_data["angle"]

        # Read the target coordinates once
        target_data = read_json(SLOW_ROBOT_COORDS_FILE)
        if not target_data:
            print("Error: Could not read target JSON data.")
            target_data = read_json(SLOW_ROBOT_COORDS_FILE)

        x_R_target = target_data["x"]
        x_target = target_data["x"] - 350
        y_target = target_data["y"]

        target_angle = calculate_angle(x_robot, y_robot, x_target, y_target)
        angle_error = (target_angle - angle_robot + 180) % 360 - 180  # Normalize to [-180, 180]
        
        # Compute distance to target
        distance = math.sqrt((x_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        distance_backwards = math.sqrt((x_R_target - x_robot) ** 2 + (y_target - y_robot) ** 2)
        if distance < distance_tolerance or distance_backwards < 300:
            print("Target reached!")
            
            
            print("Not Safe to overtake. Road not clear.")
            fa_robot.SetMotors(0, 0)  # Stop motors
            time.sleep(1)
            
        # Initialize turn_adjustment
        turn_adjustment = None

        # Check if angle difference is less than 10 degrees
        if abs(angle_error) < 15:
            left_speed = base_speed
            right_speed = base_speed
        else:
            # Compute correction from PID controller
            turn_adjustment = pid(angle_error)
            turn_adjustment = turn_adjustment / 1.5
            left_speed = base_speed - turn_adjustment
            right_speed = base_speed + turn_adjustment

        fa_robot.SetMotors(left_speed, right_speed)

        print(f"Robot Angle: {angle_robot}, Target Angle: {target_angle}, Angle Error: {angle_error}, Turn Adj: {turn_adjustment if turn_adjustment is not None else 'None'}")

        
# Ensure the function only runs when executed directly
if __name__ == "__main__":
    FA1 = FA.Create()
    FA1.ComOpen(6)

    LineFollowPID_Fast(FA1) # Call the PID line following function

 