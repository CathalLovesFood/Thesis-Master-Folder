import numpy as np
import math
import json
import os
import csv
import time
from PIDLineFollow import read_json

# Solves known issue with OpenCV as it will shorten the opening time
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import cv2.aruco as aruco

# ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
aruco_parameters = aruco.DetectorParameters()

# Set Lane positions for visual reference and for Vehicles
LANE_Y_POSITIONS = {"left_lane": 300, "right_lane": 450}

# Marker assignments for identification
robot_marker_assignments = { 
    1: "Robot Slow",
    2: "Robot Fast",
    3: "Robot Test",
}

# Function to write robot coordinates to JSON file
def WriteRobotCoordinates(x, y, angle, marker_id):
    # Mapping of marker IDs to file names
    file_mapping = {
        1: 'Automated-Overtaking/OvertakeCoordinates_Slow.json',
        2: 'Automated-Overtaking/OvertakeCoordinates_Fast.json',
        3: 'Automated-Overtaking/OvertakeCoordinates_Test.json'
    }
    
    # Determine the file name based on the marker ID
    file_name = file_mapping.get(marker_id, 'Automated-Overtaking/OvertakeCoordinates.json')
    
    data = {}
    data['robot'] = []
    data['robot'].append({
        'id': int(marker_id),   # Marker ID of Aruco marker
        'x': int(x),            # Inputs X coordinate
        'y': int(y),            # Inputs Y coordinate
        'angle': angle          # Orientation angle
    })
    
    # Write to the corresponding file
    with open(file_name, 'w') as f:
        json.dump(data, f)

# Function to write target coordinates to JSON file
def TargetCoordinates(x, y, marker_id):
    # Mapping of marker IDs to file names
    file_mapping = {
        1: 'Automated-Overtaking/TargetCoordinates_Phase2.json',
        2: 'Automated-Overtaking/TargetCoordinates_Phase2.json',
        3: 'Automated-Overtaking/TargetCoordinates_Phase1.json'
    }
    
    file_name = file_mapping.get(marker_id, 'Automated-Overtaking/TargetCoordinates.json')
    
    data = {}
    data['robot'] = []
    data['robot'].append({
        'x': x,
        'y': y,
    })
    with open(file_name, 'w') as f:
        json.dump(data, f)

#  Function specifically for Phase 3 target coordinates
def TargetCoordinatesPhase3(x, y, marker_id):
    # Mapping of marker IDs to file names
    file_mapping = {
        1: 'Automated-Overtaking/TargetCoordinates_Phase2.json',
        2: 'Automated-Overtaking/TargetCoordinates_Phase2.json',
        3: 'Automated-Overtaking/TargetCoordinates_Phase3.json'
    }
    
    file_name = file_mapping.get(marker_id, 'Automated-Overtaking/TargetCoordinates.json')
    
    data = {}
    data['robot'] = []
    data['robot'].append({
        'x': x,
        'y': y,
    })
    with open(file_name, 'w') as f:
        json.dump(data, f)

# Function to clear JSON files for marker ID 2. This is utilized for the wait to overtake fucntion as it allows the removal of the target to signify the road is clear
def clear_json_files_for_missing_markers(detected_ids):
    file_mapping = {
        2: 'Automated-Overtaking/OvertakeCoordinates_Fast.json'
    }

    # Initialize data variable to avoid scope issues
    data = {}
    
    # Set the coordinates and angle to specific values when marker is not detected
    default_x = 0  # Set the default X coordinate
    default_y = 0  # Set the default Y coordinate
    default_angle = 0  # Set the default angle

    # Iterate through file_mapping to check if markers are missing
    for marker_id, file_path in file_mapping.items():
        if marker_id not in detected_ids:
            # If marker is missing, set data with specific default values
            data['robot'] = []
            data['robot'].append({
                'id': int(marker_id),    # Marker ID of Aruco marker
                'x': default_x,          # Set specific X coordinate
                'y': default_y,          # Set specific Y coordinate
                'angle': default_angle   # Set specific angle
            })
    
            # Write to the corresponding file
            with open(file_path, 'w') as f:
                json.dump(data, f)

# Function to detect ArUco markers and get positions and orientations
def get_robot_position_and_orientation(frame):
    # Convert image to gray scale to allow easier detection of ArUco markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)
    
    marker_data = {}

    # Check if any markers are detected
    if ids is not None:
        # Cycle through the detected markers
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]  # Get the four corners of the detected marker
            
            # Compute the center (X, Y) of the marker
            x_center = int(np.mean(marker_corners[:, 0]))  
            y_center = int(np.mean(marker_corners[:, 1]))  

            # Calculate orientation using the top two corners
            top_left = marker_corners[0]  
            top_right = marker_corners[1]  

            # Compute angle using atan2
            angle = math.degrees(math.atan2(top_right[1] - top_left[1], top_right[0] - top_left[0]))
            marker_data[marker_id] = (x_center, y_center, angle)
            print(f"Marker {marker_id} at ({x_center}, {y_center}), Angle: {angle}")
            
            # Write the marker data to the respective JSON file
            WriteRobotCoordinates(x_center, y_center, angle, marker_id)
    
    # Clear missing markers from JSON files and return information
    detected_ids = set(marker_data.keys())
    clear_json_files_for_missing_markers(detected_ids)
    return marker_data

# Function to set lane positions
def set_lane(x_current, y_current, lane):
    # Determine the target position based on the lane
    target_x = x_current  
    target_y = LANE_Y_POSITIONS[lane]

    return target_x, target_y

# Function to draw debug overlay on the camera feed, Shows lane lines for consistent setup, highlights robot positions and orientations, and draws target positions
def draw_debug_overlay(frame, marker_data):
    """ Draws the lane lines, robot positions, and orientations on the camera feed. """
    height, width, _ = frame.shape

    # Draw left and right lane lines
    cv2.line(frame, (0, LANE_Y_POSITIONS["left_lane"]), (width, LANE_Y_POSITIONS["left_lane"]), (255, 0, 0), 2)
    cv2.line(frame, (0, LANE_Y_POSITIONS["right_lane"]), (width, LANE_Y_POSITIONS["right_lane"]), (0, 0, 255), 2)

    # Initialises to check if fixed target data exists for Phase 1 and Phase 3
    fixed_target_phase1 = None
    try:
        with open('Automated-Overtaking/fixed_target_data_phase1.json', 'r') as f:
            fixed_target_phase1 = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        fixed_target_phase1 = None
    
    fixed_target_phase3 = None
    try:
        with open('Automated-Overtaking/fixed_target_data_phase3.json', 'r') as f:
            fixed_target_phase3 = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        fixed_target_phase3 = None
    
    
    for marker_id, (x, y, angle) in marker_data.items():
        cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)  # Draw robot marker

        
        # Draw text info
        cv2.putText(frame, f"Marker {marker_id}: ({x}, {y})", (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        phase1_finished = False
        try:
            with open('Automated-Overtaking/phase1_complete.json', 'r') as f:
                data = json.load(f)
                phase1_finished = data.get("finished", False)
        except (FileNotFoundError, json.JSONDecodeError):
            phase1_finished = False
        
        
        if (marker_id == 3):
            # Draw target position Phase 1
            if fixed_target_phase1:
                # If a fixed target exists, draw it
                x_target, y_target = fixed_target_phase1["x"], fixed_target_phase1["y"]
                cv2.circle(frame, (int(x_target), int(y_target)), 10, (0, 255, 255), -1)
                cv2.putText(frame, f"Dynamic Target: ({int(x_target)}, {int(y_target)})", (int(x_target), int(y_target) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                cv2.putText(frame, f"Target Phase 1: ({int(x_target)}, {int(y_target)})", (int(x_target), int(y_target) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                # Otherwise, draw dynamic target
                x_target, y_target = set_lane(x, y, "left_lane")
                x_target += 200  # your custom adjustment
                TargetCoordinates(x_target, y_target, marker_id)
                cv2.circle(frame, (int(x_target), int(y_target)), 10, (255, 0, 255), -1)
                cv2.putText(frame, f"Dynamic Target: ({int(x_target)}, {int(y_target)})", (int(x_target), int(y_target) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            
            
            if phase1_finished: 
                if fixed_target_phase3:
                    # If a fixed target exists, draw it
                    x_target, y_target = fixed_target_phase3["x"], fixed_target_phase3["y"]
                    cv2.circle(frame, (int(x_target), int(y_target)), 10, (0, 255, 255), -1)
                    cv2.putText(frame, f"Target Phase 3: ({int(x_target)}, {int(y_target)})", (int(x_target), int(y_target) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                else:
                    # Otherwise, draw dynamic target
                    x_target, y_target = set_lane(x, y, "right_lane")
                    x_target += 250  # your adjustment for Phase 3 too

                    cv2.circle(frame, (int(x_target), int(y_target)), 10, (255, 0, 255), -1)
                    TargetCoordinatesPhase3(x_target, y_target, marker_id)
                    cv2.putText(frame, f"Dynamic Target: ({int(x_target)}, {int(y_target)})", (int(x_target), int(y_target) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        if(marker_id == 1):
            # Draw target position Phase 2
            x_target, y_target = set_lane(x, y, "left_lane")
            cv2.circle(frame, (int(x_target+250), int(y_target)), 10, (255, 0, 0), -1)  # Change color to blue
            TargetCoordinates(x_target+250, y_target, marker_id)
            cv2.putText(frame, f"Target Phase 2: ({int(x_target+250)}, {int(y_target)})", (int(x_target+250), int(y_target) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Draw target position for the point at which the overtake should be initiated
            x_target, y_target = set_lane(x, y, "right_lane")
            cv2.circle(frame, (int(x_target-300), int(y_target)), 10, (255, 0, 0), -1)  # Change color to blue
            cv2.putText(frame, f"Overtake Point: ({int(x_target-300)}, {int(y_target)})", (int(x_target-300), int(y_target) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            
        # Draw orientation arrow
        arrow_length = 50
        x_end = int(x + arrow_length * np.cos(math.radians(angle)))
        y_end = int(y + arrow_length * np.sin(math.radians(angle)))
        cv2.arrowedLine(frame, (x, y), (x_end, y_end), (255, 0, 0), 3)  # Blue arrow

    return frame

def create_window():
    # Open camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # File paths for logging
    logging_flag_file = "Automated-Overtaking/logging_flag.json"
    csv_file = "Automated-Overtaking/robot_coordinates.csv"

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Detect ArUco markers and get their positions and orientations
        marker_data = get_robot_position_and_orientation(frame)

        if marker_data:
            # Update the overlay with robot positions, orientations, and targets
            frame = draw_debug_overlay(frame, marker_data)

            # Check the logging flag
            try:
                with open(logging_flag_file, 'r') as f:
                    logging_flag = json.load(f).get("logging_enabled", False)
            except (FileNotFoundError, json.JSONDecodeError):
                logging_flag = False

            if logging_flag:
                # Log test marker coordinates
                test_marker_data = read_json("Automated-Overtaking/OvertakeCoordinates_Test.json")
                if test_marker_data:
                    x = test_marker_data["x"]
                    y = test_marker_data["y"]
                    angle = test_marker_data["angle"]
                    log_coordinates_to_csv(csv_file, x, y, angle, "Test Marker")

                # Log slow robot coordinates
                slow_robot_data = read_json("Automated-Overtaking/OvertakeCoordinates_Slow.json")
                if slow_robot_data:
                    x = slow_robot_data["x"]
                    y = slow_robot_data["y"]
                    angle = slow_robot_data["angle"]
                    log_coordinates_to_csv(csv_file, x, y, angle, "Slow Robot")

        # Display the frame with the overlay
        cv2.imshow("ArUco Robot Orientation", frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def log_coordinates_to_csv(file_path, x, y, angle, robot_type, timestamp=None):
    """Log coordinates and angle to a CSV file."""
    if timestamp is None:
        timestamp = time.time()

    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([robot_type, x, y, angle, timestamp])


if __name__ == "__main__":
    
    create_window()  # Call the function to start the camera feed