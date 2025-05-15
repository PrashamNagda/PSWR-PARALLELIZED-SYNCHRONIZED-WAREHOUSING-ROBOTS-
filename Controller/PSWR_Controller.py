import cv2
import cv2.aruco as aruco
import numpy as np
import math
import socket
import time
import threading
from collections import defaultdict

# ===== CONFIG =====
ROBOT_IDS = [4, 5, 6]  # IDs of the robots in the system
PRODUCT_IDS = [16, 17, 18, 19]  # These are the product/target IDs
CHARGING_STATION_IDS = [12, 13, 14, 15]  # Charging station IDs
IGNORE_IDS = []
ROBOT_IPS = {
    4: "192.168.137.107",
    5: "192.168.137.206",
    6: "192.168.137.39",
}
UDP_PORT = 8888
COMMAND_INTERVAL = 1.65
ANGLE_TOLERANCE = math.radians(8)
STOP_COMMAND_THRESHOLD = 6
FORWARD_CYCLES_BEFORE_CHECK = 2
MAX_PRODUCT_NOT_VISIBLE = 6  # For product pickup
MAX_DOCKING_NOT_VISIBLE = 6  # For docking completion
MAX_CHARGING_NOT_VISIBLE = 6  # For charging station completion
ROBOT_AVOIDANCE_DISTANCE = 40 #Distance to maintain from other robots
COLLISION_WAIT_TIME = 3  # Seconds to wait when collision occurs
REVERSE_TIME = 0.8  # Time to reverse when collision occurs (1 second)

# Docking station mapping (product_id: docking_station_id)
DOCKING_MAP = {
    16: 8,
    17: 9,
    18: 10,
    19: 11
}

# Track occupied charging stations
OCCUPIED_CHARGING_STATIONS = set()

# Predefined commands
LEFT_TURN = (85, 85)
RIGHT_TURN = (100, 100)
STOP = (90, 90)
FORWARD_COMMAND = (104, 75)
REVERSE_COMMAND = (74, 112)
ROUND_COMMAND = (10, 10)

# ===== SETUP =====
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
detector_params = aruco.DetectorParameters()
udp_sockets = {robot_id: socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for robot_id in ROBOT_IDS}

class RobotState:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.selected_product = None
        self.product_picked = False
        self.docking_complete = False
        self.charging_complete = False
        self.charging_station = None
        self.product_not_visible_count = 0
        self.docking_not_visible_count = 0
        self.charging_not_visible_count = 0
        self.last_command_time = 0
        self.alignment_achieved = False
        self.forward_counter = 0
        self.stop_command_counter = 0
        self.reached_target = False
        self.current_target_id = None
        self.robot_position = None
        self.robot_angle = None
        self.lock = threading.Lock()
        self.collision_stop_time = 0
        self.is_stopped_for_collision = False
        self.should_reverse = False
        self.reverse_start_time = 0
        self.reverse_command_sent = False

    def reset_movement_state(self):
        self.alignment_achieved = False
        self.forward_counter = 0
        self.stop_command_counter = 0
        self.reached_target = False

    def reset_for_new_task(self):
        global OCCUPIED_CHARGING_STATIONS
        
        if self.charging_station is not None:
            OCCUPIED_CHARGING_STATIONS.discard(self.charging_station)
            self.charging_station = None
            
        self.selected_product = None
        self.product_picked = False
        self.docking_complete = False
        self.charging_complete = False
        self.reset_movement_state()
        self.product_not_visible_count = 0
        self.docking_not_visible_count = 0
        self.charging_not_visible_count = 0
        self.is_stopped_for_collision = False
        self.should_reverse = False

# Initialize robot states
robot_states = {robot_id: RobotState(robot_id) for robot_id in ROBOT_IDS}
assigned_products = set()
available_products = set(PRODUCT_IDS)
frame_lock = threading.Lock()
current_frame = None
current_corners = None
current_ids = None

def get_angle(corners):
    dx = corners[0][1][0] - corners[0][0][0]
    dy = -(corners[0][1][1] - corners[0][0][1])
    return math.atan2(dy, dx)

def get_center(corners):
    return np.mean(corners[0], axis=0)

def send_command(robot_id, left_speed, right_speed, message=None):
    """Send command to robot, optionally with a message"""
    if left_speed == "M" or left_speed == "D" or left_speed == "C":
        cmd = left_speed
    elif message is not None:
        cmd = f"{int(left_speed)},{int(right_speed)},{message}"
    else:
        cmd = f"{int(left_speed)},{int(right_speed)}"
   
    udp_sockets[robot_id].sendto(cmd.encode(), (ROBOT_IPS[robot_id], UDP_PORT))
    return cmd.startswith(f"{STOP[0]},{STOP[1]}")

def check_alignment(robot_center, target_center, robot_angle):
    dx = target_center[0] - robot_center[0]
    dy = robot_center[1] - target_center[1]
    desired_angle = math.atan2(dy, dx)
    angle_error = (desired_angle - robot_angle + math.pi) % (2 * math.pi) - math.pi
    return angle_error

def is_close_enough(robot_center, target_center, threshold=50):
    distance = np.linalg.norm(np.array(robot_center) - np.array(target_center))
    return distance < threshold

def get_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def get_marker_id(id_val):
    return id_val[0] if isinstance(id_val, (np.ndarray, list)) else id_val

def find_nearest_available_robot(product_pos, corners, ids):
    available_robots = []
   
    for i, id_val in enumerate(ids):
        marker_id = get_marker_id(id_val)
        if marker_id in ROBOT_IDS:
            state = robot_states[marker_id]
            if state.selected_product is None and not state.docking_complete and not state.charging_complete:
                robot_pos = get_center(corners[i])
                distance = get_distance(product_pos, robot_pos)
                available_robots.append((distance, marker_id, robot_pos))
   
    if not available_robots:
        return None
   
    available_robots.sort()
    return available_robots[0][1]

def find_nearest_charging_station(robot_pos, corners, ids, robot_id=None):
    charging_stations = []
    
    if ids is None:
        return None
        
    marker_ids = [get_marker_id(id_val) for id_val in ids]
    
    # Robot 4 always goes to station 12 if available
    if robot_id == 4:
        if 12 in marker_ids and 12 not in OCCUPIED_CHARGING_STATIONS:
            station_idx = marker_ids.index(12)
            station_pos = get_center(corners[station_idx])
            return 12
        return None
    
    for station_id in CHARGING_STATION_IDS:
        if station_id in marker_ids and station_id not in OCCUPIED_CHARGING_STATIONS:
            station_idx = marker_ids.index(station_id)
            station_pos = get_center(corners[station_idx])
            distance = get_distance(robot_pos, station_pos)
            charging_stations.append((distance, station_id, station_pos))
    
    if not charging_stations:
        return None
    
    charging_stations.sort()
    return charging_stations[0][1]

def assign_product_to_nearest_robot(product_id, corners, ids):
    global assigned_products, available_products
   
    if product_id in assigned_products:
        print(f"Product {product_id} is already assigned!")
        return False
   
    product_pos = None
    marker_ids = [get_marker_id(id_val) for id_val in ids] if ids is not None else []
    if product_id in marker_ids:
        product_idx = marker_ids.index(product_id)
        product_pos = get_center(corners[product_idx])
   
    if product_pos is None:
        print(f"Product {product_id} not visible - cannot determine nearest robot")
        return False
   
    nearest_robot = find_nearest_available_robot(product_pos, corners, ids)
   
    if nearest_robot is not None:
        state = robot_states[nearest_robot]
        state.selected_product = product_id
        assigned_products.add(product_id)
        available_products.discard(product_id)
        docking_station_id = DOCKING_MAP.get(product_id, 8)
        print(f"Assigned product {product_id} to nearest robot {nearest_robot} (docking station {docking_station_id})")
        state.reset_movement_state()
        state.product_picked = False
        state.docking_complete = False
        state.charging_complete = False
        state.product_not_visible_count = 0
        state.docking_not_visible_count = 0
        state.charging_not_visible_count = 0
        return True
    else:
        print("No available robots to assign!")
        return False

def handle_keyboard_input(key, corners, ids):
    product_mapping = {
        ord('1'): 16,
        ord('2'): 17,
        ord('3'): 18,
        ord('4'): 19
    }
   
    if key in product_mapping:
        product_id = product_mapping[key]
        assign_product_to_nearest_robot(product_id, corners, ids)

def check_collisions(corners, ids):
    """Collision avoidance - stops both robots, then higher ID robot reverses for 1 second"""
    if ids is None:
        return
    
    current_time = time.time()
    robot_positions = {}
    
    # First collect all robot positions
    for i, id_val in enumerate(ids):
        marker_id = get_marker_id(id_val)
        if marker_id in ROBOT_IDS:
            robot_positions[marker_id] = get_center(corners[i])
    
    # Check distances between all pairs of robots
    robot_ids = sorted(robot_positions.keys())
    for i in range(len(robot_ids)):
        for j in range(i+1, len(robot_ids)):
            robot_id1 = robot_ids[i]
            robot_id2 = robot_ids[j]
            
            pos1 = robot_positions[robot_id1]
            pos2 = robot_positions[robot_id2]
            
            distance = get_distance(pos1, pos2)
            
            if distance < ROBOT_AVOIDANCE_DISTANCE * 2:  # Check if circles overlap
                state1 = robot_states[robot_id1]
                state2 = robot_states[robot_id2]
                
                # If not already in collision state
                if not state1.is_stopped_for_collision and not state2.is_stopped_for_collision:
                    print(f"COLLISION AVOIDANCE: Robots {robot_id1} and {robot_id2} stopping (distance: {distance:.1f}px)")
                    
                    # Stop both robots
                    send_command(robot_id1, *STOP)
                    send_command(robot_id2, *STOP)
                    
                    # Mark both as in collision state
                    state1.is_stopped_for_collision = True
                    state2.is_stopped_for_collision = True
                    state1.collision_stop_time = current_time
                    state2.collision_stop_time = current_time
                    
                    # Higher ID robot should reverse
                    higher_id = max(robot_id1, robot_id2)
                    robot_states[higher_id].should_reverse = True
                    robot_states[higher_id].reverse_start_time = current_time
                    robot_states[higher_id].reverse_command_sent = False  # Reset flag
                    
                    print(f"COLLISION AVOIDANCE: Robot {higher_id} will reverse")
                
                # Visual feedback
                if 'frame_debug' in locals():
                    cv2.line(frame_debug,
                            (int(pos1[0]), int(pos1[1])),
                            (int(pos2[0]), int(pos2[1])),
                            (0, 0, 255), 2)
                    cv2.circle(frame_debug, 
                              (int(pos1[0]), int(pos1[1])), 
                              ROBOT_AVOIDANCE_DISTANCE, 
                              (0, 255, 255), 2)
                    cv2.circle(frame_debug, 
                              (int(pos2[0]), int(pos2[1])), 
                              ROBOT_AVOIDANCE_DISTANCE, 
                              (0, 255, 255), 2)
    
    # Handle robots that are in collision state
    for robot_id in ROBOT_IDS:
        state = robot_states[robot_id]
        
        if state.is_stopped_for_collision:
            current_time = time.time()
            
            # If this robot should reverse and hasn't sent the command yet
            if state.should_reverse and not state.reverse_command_sent:
                send_command(robot_id, *REVERSE_COMMAND)
                state.reverse_command_sent = True
                print(f"Robot {robot_id} reversing (command sent once)")
            
            # If collision timeout has passed
            if current_time - state.collision_stop_time >= COLLISION_WAIT_TIME:
                print(f"COLLISION AVOIDANCE: Robot {robot_id} resuming operation")
                state.is_stopped_for_collision = False
                state.should_reverse = False
                state.reverse_command_sent = False
                send_command(robot_id, *STOP)  # Ensure robot is stopped before resuming
                  

def udp_listener():
    """Thread that listens for UDP messages from robots"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT + 1))  # Listen on a different port than we send to
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode().strip()
            
            # Find which robot sent this message based on IP
            robot_id = None
            for rid, ip in ROBOT_IPS.items():
                if ip == addr[0]:
                    robot_id = rid
                    break
            
            if robot_id is None:
                continue
                
            state = robot_states[robot_id]
            
            if message == "L":
                print(f"Robot {robot_id} reports LOW BATTERY - sending to charging")
                with state.lock:
                    # If currently doing a task, mark it as incomplete
                    if state.selected_product is not None:
                        assigned_products.discard(state.selected_product)
                        available_products.add(state.selected_product)
                        state.selected_product = None
                        state.product_picked = False
                    
                    # Set flags to go to charging
                    state.docking_complete = True
                    state.charging_complete = False
                    state.charging_station = None
                    state.reset_movement_state()
                    
            elif message == "C":
                print(f"Robot {robot_id} reports CHARGING COMPLETE - available for tasks")
                with state.lock:
                    state.reset_for_new_task()
                    
        except Exception as e:
            print(f"Error in UDP listener: {e}")

def handle_charging(state, robot_id, corners, ids, current_time):
    global OCCUPIED_CHARGING_STATIONS
    
    if state.charging_station is None:
        if state.robot_position is not None:
            state.charging_station = find_nearest_charging_station(state.robot_position, corners, ids, robot_id)
            if state.charging_station is not None:
                OCCUPIED_CHARGING_STATIONS.add(state.charging_station)
                print(f"Robot {robot_id}: Moving to charging station {state.charging_station}")
                state.reset_movement_state()
            else:
                print(f"Robot {robot_id}: No available charging stations")
                send_command(robot_id, *STOP)
                return True
        else:
            send_command(robot_id, *STOP)
            return True

    if current_time - state.last_command_time >= COMMAND_INTERVAL:
        if ids is not None:
            marker_ids = [get_marker_id(id_val) for id_val in ids] if ids is not None else []
            if robot_id in marker_ids:
                robot_idx = marker_ids.index(robot_id)
                robot_pos = get_center(corners[robot_idx])
                robot_angle = get_angle(corners[robot_idx])
                state.robot_position = robot_pos
                state.robot_angle = robot_angle
                
                target_id = state.charging_station
                state.current_target_id = target_id
                
                if target_id in marker_ids:
                    state.charging_not_visible_count = 0
                    target_idx = marker_ids.index(target_id)
                    target_pos = get_center(corners[target_idx])
                    
                    if state.charging_not_visible_count >= MAX_CHARGING_NOT_VISIBLE:
                        print(f"Robot {robot_id}: === REACHED CHARGING STATION {target_id} ===")
                        send_command(robot_id, *STOP)
                        time.sleep(2)
                        print(f"Robot {robot_id}: === CHARGING STARTED AT STATION {target_id} ===")
                        state.charging_complete = True
                        state.reset_for_new_task()
                        return True
                    
                    if not state.alignment_achieved:
                        angle_error = check_alignment(robot_pos, target_pos, robot_angle)
                        
                        if abs(angle_error) < ANGLE_TOLERANCE:
                            if send_command(robot_id, *STOP):
                                state.stop_command_counter += 1
                            state.alignment_achieved = True
                            state.forward_counter = 0
                            print(f"Robot {robot_id}: === ALIGNMENT ACHIEVED ===")
                        else:
                            state.stop_command_counter = 0
                            if angle_error > 0:
                                send_command(robot_id, *LEFT_TURN)
                            else:
                                send_command(robot_id, *RIGHT_TURN)
                    else:
                        send_command(robot_id, *FORWARD_COMMAND)
                        state.forward_counter += 1
                        state.stop_command_counter = 0
                        
                        if state.forward_counter >= FORWARD_CYCLES_BEFORE_CHECK:
                            state.alignment_achieved = False
                            print(f"Robot {robot_id}: === INITIATING ALIGNMENT CHECK ===")
                else:
                    state.charging_not_visible_count += 1
                    print(f"Robot {robot_id}: Charging station not visible count: {state.charging_not_visible_count}/{MAX_CHARGING_NOT_VISIBLE}")
                    
                    if state.charging_not_visible_count >= MAX_CHARGING_NOT_VISIBLE:
                        print(f"Robot {robot_id}: === ASSUMING CHARGING COMPLETE (STATION NOT VISIBLE) ===")
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(0.2)
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(0.2)
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(0.2)
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(0.2)
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(0.2)
                        send_command(robot_id, *LEFT_TURN)
                        time.sleep(1)
                        send_command(robot_id, *REVERSE_COMMAND)
                        time.sleep(0.5)
                        send_command(robot_id, *REVERSE_COMMAND)
                        send_command(robot_id, *STOP)
                        time.sleep(0.5)
                        send_command(robot_id, "C", 0)
                        state.charging_complete = True
                        state.reset_for_new_task()
                        return True
                    
                    send_command(robot_id, *STOP)
            else:
                send_command(robot_id, *STOP)
        
        state.last_command_time = current_time
    
    return False

def process_robot(robot_id, state, corners, ids, current_time):
    # Skip processing if robot is stopped for collision
    if state.is_stopped_for_collision:
        return
    
    # Handle charging state first if applicable
    if state.docking_complete and not state.charging_complete:
        if handle_charging(state, robot_id, corners, ids, current_time):
            return
    
    if state.selected_product is None or state.docking_complete or state.charging_complete:
        return
    
    if current_time - state.last_command_time >= COMMAND_INTERVAL:
        if ids is not None:
            marker_ids = [get_marker_id(id_val) for id_val in ids] if ids is not None else []
            if robot_id in marker_ids:
                robot_idx = marker_ids.index(robot_id)
                robot_pos = get_center(corners[robot_idx])
                robot_angle = get_angle(corners[robot_idx])
                state.robot_position = robot_pos
                state.robot_angle = robot_angle
               
                target_id = DOCKING_MAP.get(state.selected_product, 8) if state.product_picked else state.selected_product
                state.current_target_id = target_id
               
                if target_id in marker_ids:
                    target_idx = marker_ids.index(target_id)
                    if state.product_picked:
                        state.docking_not_visible_count = 0
                    else:
                        state.product_not_visible_count = 0
                   
                    target_pos = get_center(corners[target_idx])
                   
                    if not state.alignment_achieved:
                        angle_error = check_alignment(robot_pos, target_pos, robot_angle)
                       
                        if abs(angle_error) < ANGLE_TOLERANCE:
                            if send_command(robot_id, *STOP):
                                state.stop_command_counter += 1
                            state.alignment_achieved = True
                            state.forward_counter = 0
                            print(f"Robot {robot_id}: === ALIGNMENT ACHIEVED ===")
                        else:
                            state.stop_command_counter = 0
                            if angle_error > 0:
                                send_command(robot_id, *LEFT_TURN)
                            else:
                                send_command(robot_id, *RIGHT_TURN)
                    else:
                        send_command(robot_id, *FORWARD_COMMAND)
                        state.forward_counter += 1
                        state.stop_command_counter = 0
                       
                        if state.forward_counter >= FORWARD_CYCLES_BEFORE_CHECK:
                            state.alignment_achieved = False
                            print(f"Robot {robot_id}: === INITIATING ALIGNMENT CHECK ===")
                else:
                    if state.product_picked:
                        state.docking_not_visible_count += 1
                        print(f"Robot {robot_id}: Docking station not visible count: {state.docking_not_visible_count}/{MAX_DOCKING_NOT_VISIBLE}")
                        
                        if state.docking_not_visible_count >= MAX_DOCKING_NOT_VISIBLE:
                            print(f"Robot {robot_id}: === ASSUMING REACHED DOCKING STATION ===")
                            send_command(robot_id, *FORWARD_COMMAND)
                            send_command(robot_id, *STOP)
                            send_command(robot_id, "D", 0)
                            time.sleep(2)
                            print(f"Robot {robot_id}: === PRODUCT DOCKED ===")
                            state.docking_complete = True
                            send_command(robot_id, *REVERSE_COMMAND)
                            time.sleep(1)
                            assigned_products.discard(state.selected_product)
                            state.selected_product = None
                            state.product_picked = False
                            state.reached_target = False
                            return
                    else:
                        state.product_not_visible_count += 1
                        print(f"Robot {robot_id}: Product not visible count: {state.product_not_visible_count}/{MAX_PRODUCT_NOT_VISIBLE}")
                        
                        if state.product_not_visible_count >= MAX_PRODUCT_NOT_VISIBLE:
                            print(f"Robot {robot_id}: === ASSUMING REACHED PRODUCT ===")
                            send_command(robot_id, *STOP)
                            send_command(robot_id, "M", 0)
                            time.sleep(2)
                            print(f"Robot {robot_id}: === PRODUCT PICKED ===")
                            state.product_picked = True
                            send_command(robot_id, *REVERSE_COMMAND)
                            time.sleep(1)
                            print(f"Robot {robot_id}: === MOVING TO DOCKING STATION {DOCKING_MAP.get(state.selected_product, 8)} ===")
                            state.reset_movement_state()
                            return
                   
                    send_command(robot_id, *STOP)
            else:
                send_command(robot_id, *STOP)
 
        state.last_command_time = current_time
        
def update_frame():
    global current_frame, current_corners, current_ids
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                current_frame = frame.copy()
                current_corners, current_ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=detector_params)

def robot_control_loop():
    while True:
        with frame_lock:
            if current_frame is None:
                time.sleep(0.1)
                continue
           
            frame_debug = current_frame.copy()
            corners = current_corners
            ids = current_ids
       
        current_time = time.time()
       
        if corners is not None and ids is not None:
            check_collisions(corners, ids)
           
            threads = []
            for robot_id, state in robot_states.items():
                t = threading.Thread(target=process_robot, args=(robot_id, state, corners, ids, current_time))
                threads.append(t)
                t.start()
           
            for t in threads:
                t.join()
       
        with frame_lock:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame_debug, corners, ids)
               
                for robot_id in ROBOT_IDS:
                    marker_ids = [get_marker_id(id_val) for id_val in ids] if ids is not None else []
                    if robot_id in marker_ids:
                        robot_idx = marker_ids.index(robot_id)
                        robot_pos = get_center(corners[robot_idx])
                       
                        # Draw 50px circle around robot
                        cv2.circle(frame_debug, 
                                  (int(robot_pos[0]), int(robot_pos[1])), 
                                  ROBOT_AVOIDANCE_DISTANCE, 
                                  (0, 255, 255), 2)
                        
                        state = robot_states[robot_id]
                        
                        # Draw path to current target
                        if state.docking_complete and not state.charging_complete:
                            if state.charging_station in marker_ids:
                                target_idx = marker_ids.index(state.charging_station)
                                target_pos = get_center(corners[target_idx])
                                cv2.line(frame_debug,
                                         (int(robot_pos[0]), int(robot_pos[1])),
                                         (int(target_pos[0]), int(target_pos[1])),
                                         (255, 0, 255), 2)
                        elif state.selected_product is not None:
                            target_id = DOCKING_MAP.get(state.selected_product, 8) if state.product_picked else state.selected_product
                            if target_id in marker_ids:
                                target_idx = marker_ids.index(target_id)
                                target_pos = get_center(corners[target_idx])
                               
                                cv2.line(frame_debug,
                                         (int(robot_pos[0]), int(robot_pos[1])),
                                         (int(target_pos[0]), int(target_pos[1])),
                                         (0, 255, 0), 2)
                               
                        # Display status text
                        status_text = f"Robot {robot_id}: "
                        if state.docking_complete and not state.charging_complete:
                            status_text += f"Charging to: {state.charging_station if state.charging_station else 'finding'}"
                            if state.charging_not_visible_count >= MAX_CHARGING_NOT_VISIBLE:
                                status_text += " (CHARGING)"
                        elif state.product_picked:
                            status_text += f"Docking to: {DOCKING_MAP.get(state.selected_product, 8)}"
                            if state.docking_not_visible_count >= MAX_DOCKING_NOT_VISIBLE:
                                status_text += " (DOCKING)"
                        elif state.selected_product is not None:
                            status_text += f"Targeting product: {state.selected_product}"
                            if state.product_not_visible_count >= MAX_PRODUCT_NOT_VISIBLE:
                                status_text += " (PICKING)"
                        else:
                            status_text += "Available"
                       
                        if not state.alignment_achieved and state.selected_product is not None:
                            status_text += " | Aligning"
                        elif state.alignment_achieved and not state.reached_target:
                            status_text += " | Moving"
                       
                        if state.is_stopped_for_collision:
                            status_text += " | STOPPED (COLLISION)"
                            if state.should_reverse:
                                status_text += " (REVERSING)"
                       
                        cv2.putText(frame_debug, status_text, (10, 30 * (robot_id - ROBOT_IDS[0] + 1)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                                    (255, 0, 255) if (state.docking_complete and not state.charging_complete) else (0, 255, 0), 
                                    2)
               
                # Display available robots count
                available_count = sum(1 for robot_id in ROBOT_IDS
                                    if robot_states[robot_id].selected_product is None and
                                    not robot_states[robot_id].docking_complete and
                                    not robot_states[robot_id].charging_complete)
                cv2.putText(frame_debug, f"Available robots: {available_count}/{len(ROBOT_IDS)}",
                            (10, frame_debug.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
               
                # Display occupied charging stations
                cv2.putText(frame_debug, f"Occupied charging stations: {', '.join(map(str, sorted(OCCUPIED_CHARGING_STATIONS)))}",
                            (10, frame_debug.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
               
                cv2.imshow("Control", frame_debug)
       
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key in [ord('1'), ord('2'), ord('3'), ord('4')]:
            with frame_lock:
                handle_keyboard_input(key, corners, ids)

def main():
    print("System ready - press 1-4 to assign products to robots")
    print("1: Product 16 | 2: Product 17 | 3: Product 18 | 4: Product 19")
    print("Charging stations: 12, 13, 14, 15")
   
    frame_thread = threading.Thread(target=update_frame)
    frame_thread.daemon = True
    frame_thread.start()
   
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()
    
    # Start the UDP listener thread
    udp_thread = threading.Thread(target=udp_listener)
    udp_thread.daemon = True
    udp_thread.start()
   
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for robot_id in ROBOT_IDS:
            send_command(robot_id, *STOP)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()