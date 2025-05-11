import serial
import time
import FA
import threading
import math
import os
import json
from movetopoint_degree import read_json, Overtake_P2P
from PIDLineFollow import LineFollowPID_Fast

if __name__ == "__main__":
    # Initialize all code connection
    FA1 = FA.Create()
    FA1.ComOpen(6)

    # Set the JSON flag to enable logging
    logging_flag_file = "Automated-Overtaking/logging_flag.json"
    with open(logging_flag_file, 'w') as f:
        json.dump({"logging_enabled": True}, f)

    print("System started. Logging coordinates...")

    OVERTAKE_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Test.json"
    SLOW_COORDS_FILE = "Automated-Overtaking/OvertakeCoordinates_Slow.json"
    
    distance_tolerance = 200  # Distance within which the robot considers initiating Overtake
    overtake_initiated = False  # Flag to track if overtake has been initiated


    try:
        os.remove('Automated-Overtaking/fixed_target_data_phase1.json')
        os.remove('Automated-Overtaking/fixed_target_data_phase3.json')
        os.remove('Automated-Overtaking/phase1_complete.json')
        print("File deleted successfully!")
    except FileNotFoundError:
        print("File not found, nothing to delete.")
            
    while True:
        # Read robot positions
        Overtake_robot_data = read_json(OVERTAKE_COORDS_FILE)
        if not Overtake_robot_data:
            print("Error: Could not read target JSON data.")
            continue  # Retry reading
        
        Slow_robot_data = read_json(SLOW_COORDS_FILE)
        if not Slow_robot_data:
            print("Error: Could not read target JSON data.")
            continue  # Retry reading
        
        x_overtake = Overtake_robot_data["x"]
        y_overtake = Overtake_robot_data["y"]
        
        x_slow = Slow_robot_data["x"]
        y_slow = Slow_robot_data["y"]
        
        # Calculate distance
        distance = abs(x_slow - x_overtake)
        print(f"Distance: {distance}")
        
        LineFollowPID_Fast(FA1)
        
        # Transition to overtake maneuver if within tolerance and not already initiated
        print("Initiating overtake maneuver...")
        Overtake_P2P(FA1)
        
        FA1.SetMotors(0, 0)  # Stop motors after maneuver
        
        time.sleep(30)




