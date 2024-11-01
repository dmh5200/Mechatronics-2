import time
import threading
from pynput import keyboard
import socket
from skimage import measure
import paho.mqtt.client as mqtt

import numpy as np
import heapq
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import math

import tkinter as tk

# Define the MQTT topics
target_topic_x = "Wall-E/targets/target{}_x"
target_topic_y = "Wall-E/targets/target{}_y"
obstacle_topic_x = "Wall-E/obstacles/obstacle{}_x"
obstacle_topic_y = "Wall-E/obstacles/obstacle{}_y"
eva_topic_x = "Wall-E/events/eva_x"
eva_topic_y = "Wall-E/events/eva_y"
plant_topic_x = "Wall-E/events/plant_x"
plant_topic_y = "Wall-E/events/plant_y"
status_topic = "Wall-E/path_status"
state1to2_topic = "Wall-E/state1to2"
state2to3_topic = "Wall-E/state2to3"
state3to4_topic = "Wall-E/state3to4"
robot_angle_topic = "Wall-E/robot/angle"
corners_topic_x = "Wall-E/corners/corner{}_x"
corners_topic_y = "Wall-E/corners/corner{}_y"

# Initialize dictionaries to store targets, obstacles, grid corners, EVA, and plant coordinates
targets_dict = {}
obstacles_dict = {}
eva_dict = {}
plant_dict = {}
corners_dict = {}
# Initialize a dictionary to store the status of the path, state, and state transitions
status_dict = {'path_status': 0, 'state': 1, 'state1to2': 0, 'state2to3': 0, 'state3to4': 0}
# Initialize a dictionary to store the robot angle
robot_angle_dict = {'angle': 0}

# Event to signal state changes
state_change_event = threading.Event()

# MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to target topics
    for i in range(1, 11):  # Assuming a maximum of 10 targets
        client.subscribe(target_topic_x.format(i))
        client.subscribe(target_topic_y.format(i))
    print("Subscribed to target topics")

    for i in range(1, 11):  # Assuming a maximum of 10 obstacles
        client.subscribe(obstacle_topic_x.format(i))
        client.subscribe(obstacle_topic_y.format(i))
    print("Subscribed to obstacle topics")

    for i in range(1, 5):  # Assuming a maximum of 4 corners
        client.subscribe(corners_topic_x.format(i))
        client.subscribe(corners_topic_y.format(i))
    print("Subscribed to corner topics")

    # Monitor EVA and plant events
    client.subscribe(eva_topic_x)
    client.subscribe(eva_topic_y)
    client.subscribe(plant_topic_x)
    client.subscribe(plant_topic_y)

    # Monitor target reached status
    client.subscribe(status_topic)
    print("Subscribed to target reached status")

    # Monitor state transitions
    client.subscribe(state1to2_topic)
    client.subscribe(state2to3_topic)
    client.subscribe(state3to4_topic)

    # Monitor robot angle
    client.subscribe(robot_angle_topic)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload_str = msg.payload.decode().strip()
    
    #print(f"Received message on topic {topic}: {payload_str}")
    
    if payload_str:
        try:
            payload = float(payload_str)  # Convert to float instead of int
        except ValueError:
            print(f"Invalid payload for topic {topic}: {payload_str}")
            return
        
        if "target" in topic:
            try:
                # Split the topic to extract target number and coordinate type
                parts = topic.split('/')
                target_part = parts[-1]  # Get the last part of the topic
                target_num = int(target_part.split('_')[0].replace('target', ''))
                coord_type = target_part.split('_')[1]
            except (IndexError, ValueError) as e:
                print(f"Error parsing topic {topic}: {e}")
                return
            
            if target_num not in targets_dict:
                targets_dict[target_num] = {'x': None, 'y': None}
            
            targets_dict[target_num][coord_type] = payload
            #print(f"Received {coord_type} for target {target_num}: {payload}")
        elif "obstacle" in topic:
            try:
                # Split the topic to extract obstacle number and coordinate type
                parts = topic.split('/')
                obstacle_part = parts[-1]  # Get the last part of the topic
                obstacle_num = int(obstacle_part.split('_')[0].replace('obstacle', ''))
                coord_type = obstacle_part.split('_')[1]
            except (IndexError, ValueError) as e:
                print(f"Error parsing topic {topic}: {e}")
                return
            
            if obstacle_num not in obstacles_dict:
                obstacles_dict[obstacle_num] = {'x': None, 'y': None}
            
            obstacles_dict[obstacle_num][coord_type] = payload
            #print(f"Received {coord_type} for obstacle {obstacle_num}: {payload}")
        elif "corner" in topic:
            try:
                # Split the topic to extract corner number and coordinate type
                parts = topic.split('/')
                corner_part = parts[-1]  # Get the last part of the topic
                corner_num = int(corner_part.split('_')[0].replace('corner', ''))
                coord_type = corner_part.split('_')[1]
            except (IndexError, ValueError) as e:
                print(f"Error parsing topic {topic}: {e}")
                return
            
            if corner_num not in corners_dict:
                corners_dict[corner_num] = {'x': None, 'y': None}
            
            corners_dict[corner_num][coord_type] = payload
            #print(f"Received {coord_type} for corner {corner_num}: {payload}")
        elif "path_status" in topic:
            if payload_str:  # Check if the payload is not empty
                try:
                    status = int(payload_str)
                    if status in [0, 1]:
                        status_dict['path_status'] = status
                        print(f"Received target_reached_status: {status}")
                    else:
                        print(f"Invalid target_reached_status value: {payload_str}")
                except ValueError:
                    print(f"Invalid payload for target_reached_status: {payload_str}")
            else:
                print(f"Empty payload for topic {topic}")

        elif "state1to2" in topic:
            if payload_str:
                try:
                    status = int(payload_str)
                    if status in [0, 1]:
                        status_dict['state1to2'] = status
                        handle_state_change() # Handle state change
                        print(f"Received state1to2 status: {status}")
                    else:
                        print(f"Invalid state1to2 status value: {payload_str}")
                except ValueError:
                    print(f"Invalid payload for state1to2 status: {payload_str}")
        elif "state2to3" in topic:
            if payload_str:
                try:
                    status = int(payload_str)
                    if status in [1]:
                        status_dict['state2to3'] = status
                        handle_state_change() # Handle state change
                        print(f"Received state2to3 status: {status}")
                    else:
                        print(f"Invalid state2to3 status value: {payload_str}")
                except ValueError:
                    print(f"Invalid payload for state2to3 status: {payload_str}")
        elif "state3to4" in topic:
            if payload_str:
                try:
                    status = int(payload_str)
                    if status in [1]:
                        status_dict['state3to4'] = status
                        handle_state_change() # Handle state change
                        print(f"Received state3to4 status: {status}")
                    else:
                        print(f"Invalid state3to4 status value: {payload_str}")
                except ValueError:
                    print(f"Invalid payload for state3to4 status: {payload_str}")
        elif "events" in topic:
            try:
                # Split the topic to extract EVA or plant type
                parts = topic.split('/')
                eva_part = parts[-1]  # Get the last part of the topic
                eva_type = eva_part.split('_')[0]
                coord_type = eva_part.split('_')[1]
            except (IndexError, ValueError) as e:
                print(f"Error parsing topic {topic}: {e}")
                return
            if eva_type == "eva":
                eva_dict[coord_type] = {'x': None, 'y': None}
                eva_dict[coord_type] = payload
                #print(f"Received {coord_type} for EVA: {payload}")
            elif eva_type == "plant":
                plant_dict[coord_type] = {'x': None, 'y': None}
                plant_dict[coord_type] = payload
                #print(f"Received {coord_type} for plant: {payload}")
        elif "angle" in topic:
            if payload_str:
                try:
                    robot_angle_dict['angle'] = float(payload)
                    #print(f"Received robot angle: {payload}")
                except ValueError:
                    print(f"Invalid payload for robot angle: {payload_str}")
    else:
        print(f"Empty payload for topic {topic}")

def convert_to_simulink_format(array, MQTT_topic):

    for element in array:
        # Convert the element to the required format
        sign = np.uint8(0) if element >= 0 else np.uint8(1)  # 0 for positive, 1 for negative
        abs_element = abs(element)
        int_part_high = np.uint8(abs_element // 100)
        int_part_low = np.uint8((abs_element % 100) // 1)
        decimal_part = np.uint8((abs_element % 1) * 100)

        data = str(f"{sign}, {int_part_high}, {int_part_low}, {decimal_part}")


        # Convert numpy array to bytearray
        #uint8_data_bytearray = bytearray(int_data_np)

        # Publish the uint8 array as bytearray
        client.publish(MQTT_topic, data, qos=2, retain=True)


# Index for selecting the target to navigate to
i = 0

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
   

# Set the username and password
client.username_pw_set("student", password="HousekeepingGlintsStreetwise")

# Connect to the server using a specific port with a timeout delay (in seconds)
client.connect("fesv-mqtt.bath.ac.uk", 31415, 60)

# Start the MQTT client loop in a separate thread
mqtt_thread = threading.Thread(target=client.loop_forever)
mqtt_thread.start()

# Define the function to send the MQTT message
def send_state1to2_message():
    client.publish("Wall-E/state1to2", payload='1', qos=2, retain=True)

# Create the Tkinter GUI
root = tk.Tk()
root.title("State Machine Controller")

# Create a button that sends the MQTT message when pressed
button = tk.Button(root, text="Switch to State 2", command=send_state1to2_message)
button.pack(pady=20)

# Publish the initial state transition topics
client.publish("Wall-E/state1to2", payload=None, retain=True)
client.publish("Wall-E/state2to3", payload=None, retain=True)
client.publish("Wall-E/state3to4", payload=None, retain=True)
client.publish("Wall-E/path_status", payload=None, retain=True)
client.publish("Wall-E/position_controller/coord_reached", payload=None, retain=True)

# Define the heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Define the function to find neighbors
def get_neighbors(node, grid):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    for direction in directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1])
        if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
            if not grid[neighbor[0], neighbor[1]]:  # Not an obstacle
                neighbors.append(neighbor)
    return neighbors

# Define the A* algorithm
def a_star(grid, start, target):
    open_set = []
    heapq.heappush(open_set, (0, tuple(start)))
    came_from = {}
    g_score = {tuple(start): 0}
    f_score = {tuple(start): heuristic(start, target)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == tuple(target):
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(tuple(start))
            path.reverse()
            return np.array(path)

        for neighbor in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + 1  # Assuming uniform cost for each step
            neighbor_tuple = tuple(neighbor)
            if tentative_g_score < g_score.get(neighbor_tuple, float('inf')):
                came_from[neighbor_tuple] = current
                g_score[neighbor_tuple] = tentative_g_score
                f_score[neighbor_tuple] = tentative_g_score + heuristic(neighbor, target)
                if neighbor_tuple not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor_tuple], neighbor))

    return np.array([])  # Return an empty path if no path is found

# Define the Bresenham's line algorithm
def bresenham(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return points

# Define the line of sight function
def line_of_sight(p1, p2, obstacles_buff):
    x1, y1 = p1
    x2, y2 = p2
    points = bresenham(x1, y1, x2, y2)
    for x, y in points:
        if obstacles_buff[x, y]:
            return False
    return True

# Define the path simplification function
def simplify_path(path, obstacles_buff):
    if len(path) < 3:
        return path
    simplified_path = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i:
            if line_of_sight(path[i], path[j], obstacles_buff):
                simplified_path.append(path[j])
                i = j
                break
            j -= 1
    return np.array(simplified_path)

def calculate_position_error_state_2(target_no, start_path_position, start_target_position):

    # Publish signal to let Wall-E move out of stationary state    
    client.publish('Wall-E/position_controller/coord_reached', 2)

    while True:

        # Find the next sequential key in the dictionary
        target_keys = sorted(targets_dict.keys())
        if target_no < len(target_keys):
            next_target_key = target_keys[target_no]
        else:
            next_target_key = target_keys[0]  # Wrap around to the first key if out of bounds
        
        # Calculate the error
        # Equation 1
        eqn1_x = start_target_position[0] - math.floor(targets_dict[next_target_key]['x']) # pull current target position from MQTT and round to 0 decimal places
        eqn1_y = start_target_position[1] - math.floor(targets_dict[next_target_key]['y'])

        # Equation 2
        eqn2_x = start_path_position[0] - eqn1_x
        eqn2_y = start_path_position[1] - eqn1_y

        # Convert cartesian error to polar error
        error_dist = math.sqrt(eqn2_x**2 + eqn2_y**2)
        error_theta = math.degrees(math.atan2(eqn2_x, eqn2_y)) - robot_angle_dict['angle']
        # normalize the angle to be between -180 and 180
        if error_theta > 180:
            error_theta -= 360
        elif error_theta < -180:
            error_theta += 360 


        # Publish the error to the position controller topic
        error_dist_topic = "Wall-E/position_controller/error/dist"
        error_theta_topic = "Wall-E/position_controller/error/theta"
        convert_to_simulink_format([error_dist], error_dist_topic)
        convert_to_simulink_format([error_theta], error_theta_topic)

        if abs(error_dist) < 20: # adjust the threshold as needed
            client.publish('Wall-E/position_controller/coord_reached', 1)
            break

        # Check if state has changed
        if status_dict['state'] != 2:
            break
        
        time.sleep(0.01)  # Frequency of 100 Hz   

def calculate_position_error_state_3(target_no, start_path_position, start_target_position):

    # Publish signal to let Wall-E move out of stationary state    
    client.publish('Wall-E/position_controller/coord_reached', 2)

    while True:
        
        # Calculate the error
        # Equation 1
        eqn1_x = start_target_position[0] - math.floor(eva_dict['x']) # pull current target position from MQTT and round to 0 decimal places
        eqn1_y = start_target_position[1] - math.floor(eva_dict['y'])

        # Equation 2
        eqn2_x = start_path_position[0] - eqn1_x
        eqn2_y = start_path_position[1] - eqn1_y

        # Convert cartesian error to polar error
        error_dist = math.sqrt(eqn2_x**2 + eqn2_y**2)
        error_theta = math.degrees(math.atan2(eqn2_x, eqn2_y)) - robot_angle_dict['angle']
        # normalize the angle to be between -180 and 180
        if error_theta > 180:
            error_theta -= 360
        elif error_theta < -180:
            error_theta += 360 


        # Publish the error to the position controller topic
        error_dist_topic = "Wall-E/position_controller/error/dist"
        error_theta_topic = "Wall-E/position_controller/error/theta"
        convert_to_simulink_format([error_dist], error_dist_topic)
        convert_to_simulink_format([error_theta], error_theta_topic)

        if abs(error_dist) < 20: # adjust the threshold as needed
            client.publish('Wall-E/position_controller/coord_reached', 1)
            break

        # Check if state has changed
        if status_dict['state'] != 3:
            break

        
        time.sleep(0.01)  # Frequency of 100 Hz  

def calculate_position_error_state_4(target_no, start_path_position, start_target_position):

    # Publish signal to let Wall-E move out of stationary state    
    client.publish('Wall-E/position_controller/coord_reached', 2)

    while True:
        
        # Calculate the error
        # Equation 1
        eqn1_x = start_target_position[0] - math.floor(plant_dict['x']) # pull current target position from MQTT and round to 0 decimal places
        eqn1_y = start_target_position[1] - math.floor(plant_dict['y'])

        # Equation 2
        eqn2_x = start_path_position[0] - eqn1_x
        eqn2_y = start_path_position[1] - eqn1_y

        # Convert cartesian error to polar error
        error_dist = math.sqrt(eqn2_x**2 + eqn2_y**2)
        error_theta = math.degrees(math.atan2(eqn2_x, eqn2_y)) - robot_angle_dict['angle']
        # normalize the angle to be between -180 and 180
        if error_theta > 180:
            error_theta -= 360
        elif error_theta < -180:
            error_theta += 360 


        # Publish the error to the position controller topic
        error_dist_topic = "Wall-E/position_controller/error/dist"
        error_theta_topic = "Wall-E/position_controller/error/theta"
        convert_to_simulink_format([error_dist], error_dist_topic)
        convert_to_simulink_format([error_theta], error_theta_topic)

        if abs(error_dist) < 20: # adjust the threshold as needed
            client.publish('Wall-E/position_controller/coord_reached', 1)
            break

        # Check if state has changed
        if status_dict['state'] != 4:
            break
        
        time.sleep(0.01)  # Frequency of 100 Hz    

# Define the handle state change function
def handle_state_change():
    if status_dict['state'] == 1 and status_dict['state1to2'] == 1:
        new_state = 2
    elif status_dict['state'] == 2 and status_dict['state2to3'] == 1:
        new_state = 3
    elif status_dict['state'] == 3 and status_dict['state3to4'] == 1:
        new_state = 4
    else:
        new_state = 1

    if new_state != status_dict['state']:
        status_dict['state'] = new_state
        state_change_event.set()  # Signal the state change event

    return new_state

def keyboard_listener():
    # Key status dictionary
    key_status = {
        'up': 0,
        'down': 0,
        'left': 0,
        'right': 0
    }

    # Define key press and release handlers
    def on_press(key):
        try:
            if key == keyboard.Key.up:
                key_status['up'] = 1
                client.publish('Wall-E/up', 1)
            elif key == keyboard.Key.down:
                key_status['down'] = 1
                client.publish('Wall-E/down', 1)
            elif key == keyboard.Key.left:
                key_status['left'] = 1
                client.publish('Wall-E/left', 1)
            elif key == keyboard.Key.right:
                key_status['right'] = 1
                client.publish('Wall-E/right', 1)
        except AttributeError:
            pass

    def on_release(key):
        try:
            if key == keyboard.Key.up:
                key_status['up'] = 0
                client.publish('Wall-E/up', 0)
            elif key == keyboard.Key.down:
                key_status['down'] = 0
                client.publish('Wall-E/down', 0)
            elif key == keyboard.Key.left:
                key_status['left'] = 0
                client.publish('Wall-E/left', 0)
            elif key == keyboard.Key.right:
                key_status['right'] = 0
                client.publish('Wall-E/right', 0)
        except AttributeError:
            pass

    # Start listening to keyboard events
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


def state_machine():
    # Initial state setup
    current_state = status_dict['state']
    
    while True:  
        if current_state == 1:

            # Start the keyboard listener in a separate thread
            listener_thread = threading.Thread(target=keyboard_listener)
            listener_thread.start()            

            status_dict['state'] = 1
            print("State 1: Executing state 1 logic")

            while status_dict['state'] == 1:
                time.sleep(0.1)  # Short sleep to avoid busy-waiting

                # Check for state change
                if status_dict['state'] != 1 or state_change_event.is_set():
                    print("State change detected")
                    break
        
        elif status_dict['state'] == 2:

            status_dict['state'] = 2

            print("State 2: Executing state 2 logic")

            target_no = 0
            # state 2 logic
            while status_dict['state'] == 2:
                time.sleep(0.1)  # Short sleep to avoid busy-waiting           
             
                while True:

                    # Check if state has changed
                    if status_dict['state'] != 2:
                        break                    

                    # Wait for data to be received
                    time.sleep(1)  # Adjust the sleep time as needed

                    # Extract targets and obstacles from dictionaries
                    targets = []
                    obstacles = []
                    per_obstacles = []

                    # Define the buffer size
                    buff_size = 30  # Example buffer size

                    for key in sorted(targets_dict.keys()):
                        if targets_dict[key]['x'] is not None and targets_dict[key]['y'] is not None:
                            targets.append([int(targets_dict[key]['x']), int(targets_dict[key]['y'])])

                    for key in sorted(obstacles_dict.keys()):
                        if obstacles_dict[key]['x'] is not None and obstacles_dict[key]['y'] is not None:
                            obstacles.append([int(obstacles_dict[key]['x']), int(obstacles_dict[key]['y'])])

                    for key in sorted(eva_dict.keys()):
                        if eva_dict['x'] is not None and eva_dict['y'] is not None:
                            obstacles.append([int(eva_dict['x']), int(eva_dict['y'])])

                    for key in sorted(plant_dict.keys()):
                        if plant_dict['x'] is not None and plant_dict['y'] is not None:
                            obstacles.append([int(plant_dict['x']), int(plant_dict['y'])])

                    for key in sorted(corners_dict.keys()):
                        if corners_dict[key]['x'] is not None and corners_dict[key]['y'] is not None:
                            per_obstacles.append([int(corners_dict[key]['x']), int(corners_dict[key]['y'])])

                    if len(corners_dict) == 4:

                        # Convert the corners of the grid to a perimeter 
                        corners = np.array([[corners_dict[key]['x'], corners_dict[key]['y']] for key in sorted(corners_dict.keys())])
                        corners = corners.flatten().reshape(-1, 2)
                        corners = corners.tolist()
                        corners = np.array(corners)
                        
                        # Function to reorder corners to form a rectangle
                        def reorder_corners(corners):
                            # Calculate the centroid
                            centroid = np.mean(corners, axis=0)
                            
                            # Calculate the angle of each point relative to the centroid
                            angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])
                            
                            # Sort the points by angle
                            sorted_indices = np.argsort(angles)
                            sorted_corners = corners[sorted_indices]
                            
                            return sorted_corners

                        # Reorder the corners
                        sorted_corners = reorder_corners(corners)

                        # Close the loop
                        perimeter = np.vstack([sorted_corners, sorted_corners[0]])

                        for i in range(len(perimeter) - 1):
                            start = perimeter[i]
                            end = perimeter[i + 1]
                            distance = np.linalg.norm(end - start)
                            num_points = int(distance)
                            for j in range(num_points + 1):
                                point = start + (end - start) * (j / num_points)
                                per_obstacles.append(point)
                        

                    # Convert to numpy arrays for further processing
                    targets = np.array(targets)
                    obstacles = np.array(obstacles)
                    per_obstacles = np.array(per_obstacles)

                    # Find the number of targets detected
                    I = len(targets)

                    if target_no >= I:
                        target_no = 0
                    else:
                        target_no = target_no


                    # Start and shift values
                    start = np.array([0, 0])
                    shift_x, shift_y = 500, 500  # Example shift values

                    # Shift targets, obstacles, and start position
                    shifted_targets = targets + [shift_x, shift_y]
                    shifted_targets = shifted_targets.flatten().reshape(-1, 2)
                    shifted_obstacles = obstacles + [shift_x, shift_y]
                    shifted_obstacles = shifted_obstacles.flatten().reshape(-1, 2)
                    shifted_per_obstacles = per_obstacles + [shift_x, shift_y]
                    shifted_per_obstacles = shifted_per_obstacles.flatten().reshape(-1, 2)
                    shifted_start = start + [shift_x, shift_y]
                    shifted_start = shifted_start.flatten()

                    # Make sure all obstacle coordinates are integers
                    shifted_obstacles = shifted_obstacles.astype(int)
                    shifted_per_obstacles = shifted_per_obstacles.astype(int)

                    # Create a grid and mark obstacles
                    grid_size = (1000, 1000)
                    grid = np.zeros(grid_size, dtype=bool)

                    # Check if obstacles are within the grid
                    try:
                        for obs in shifted_obstacles:
                            grid[obs[0], obs[1]] = True
                    except IndexError:
                        print("Obstacle out of grid bounds")
                        time.sleep(1)
                        break

                    # Create a buffer around obstacles
                    obstacles_buff = binary_dilation(grid, iterations=buff_size)

                    # Dilate the perimeter obstacles
                    perimeter_buff = np.zeros_like(obstacles_buff)
                    for obs in shifted_per_obstacles:
                        perimeter_buff[obs[0], obs[1]] = True
                    perimeter_buff = binary_dilation(perimeter_buff, iterations=2)

                    # Combine perimeter with obstacles buffer
                    obstacles_buff = np.logical_or(obstacles_buff, perimeter_buff)

                    # Old method of combining obstacles and perimeter
                    #for obs in shifted_per_obstacles:
                    #    obstacles_buff[obs[0], obs[1]] = True

                    # Example usage
                    start = tuple(shifted_start)
                    target = tuple(shifted_targets[target_no])

                    # Check if the start is within an obstacle buffer
                    if obstacles_buff[start[0], start[1]]:
                        raise ValueError("Start position is within an obstacle buffer")
                    
                    if obstacles_buff[start[0], start[1]] or obstacles_buff[target[0], target[1]]:
                        plt.imshow(obstacles_buff.T, cmap='gray_r')
                        plt.scatter(shifted_start[0], shifted_start[1], c='blue', marker='o', label='Start')
                        plt.scatter(shifted_targets[:, 0], shifted_targets[:, 1], c='green', marker='x', label='Target')
                        plt.legend()
                        plt.gca().invert_yaxis()  # Flip the y-axis
                        plt.show()
                        raise ValueError("Start or target position is within an obstacle buffer")
                    
                    # Check if the target is within an obstacle buffer
                    if obstacles_buff[target[0], target[1]]:
                        raise ValueError("Target position is within an obstacle buffer")

                    
                    
                    path = a_star(obstacles_buff, start, target)
                    print("Path:", path)

                    # Simplify the path
                    simplified_path = simplify_path(path, obstacles_buff)
                    print("Simplified Path:", simplified_path)

                    # Shift the path back to the original coordinates
                    shifted_back_path = simplified_path - [shift_x, shift_y]
                    print("Shifted Back Path:", shifted_back_path)

                    # Plotting the path
                    plt.imshow(obstacles_buff.T, cmap='gray_r')
                    if simplified_path.size > 0:
                        simplified_path = np.array(simplified_path)
                        plt.plot(simplified_path[:, 0], simplified_path[:, 1], 'r-', linewidth=2)
                    plt.scatter(shifted_start[0], shifted_start[1], c='blue', marker='o', label='Start')
                    plt.scatter(shifted_targets[:, 0], shifted_targets[:, 1], c='green', marker='x', label='Target')
                    plt.legend()
                    plt.gca().invert_yaxis()  # Flip the y-axis
                    plt.show()

                    for path_no in range(len(shifted_back_path) - 1):
                        
                        # Calculate the position state error
                        start_path_position = shifted_back_path[path_no + 1] # First path position is always 0,0 - skip this
                        start_target_position = shifted_back_path[-1]
                    
                        calculate_position_error_state_2(target_no, start_path_position, start_target_position)

                    # Move on to next target 
                    target_no += 1
                    
                    # Check if state has changed
                    if status_dict['state'] != 2:
                        break
                    time.sleep(1)  # Adjust the sleep time as needed
                    print("Waiting for target reached status")
                    
                    print("Target reached - continuing to the next target")
                    status_dict['path_status'] = 0  # Reset the target reached status
        
        elif status_dict['state'] == 3:

            status_dict['state'] = 3
            
            print("State 3: Executing state 3 logic")
            # state 3 logic 

            # Initialise error values
            client.publish('Wall-E/position_controller/error/dist', payload=None, qos=2, retain=True)
            client.publish('Wall-E/position_controller/error/theta', payload=None, qos=2, retain=True)

            target_no = 0

            while status_dict['state'] == 3:
                time.sleep(0.1)  # Short sleep to avoid busy-waiting

                # Extract targets and obstacles from dictionaries
                targets = []
                obstacles = []
                per_obstacles = []

                for key in sorted(eva_dict.keys()):
                    if eva_dict['x'] is not None and eva_dict['y'] is not None:
                        targets.append([int(eva_dict['x']), int(eva_dict['y'])])

                for key in sorted(obstacles_dict.keys()):
                    if obstacles_dict[key]['x'] is not None and obstacles_dict[key]['y'] is not None:
                        obstacles.append([int(obstacles_dict[key]['x']), int(obstacles_dict[key]['y'])])

                for key in sorted(plant_dict.keys()):
                    if plant_dict['x'] is not None and plant_dict['y'] is not None:
                        obstacles.append([int(plant_dict['x']), int(plant_dict['y'])])

                for key in sorted(corners_dict.keys()):
                    if corners_dict[key]['x'] is not None and corners_dict[key]['y'] is not None:
                        per_obstacles.append([int(corners_dict[key]['x']), int(corners_dict[key]['y'])])

                if len(corners_dict) == 4:

                    # Convert the corners of the grid to a perimeter 
                    corners = np.array([[corners_dict[key]['x'], corners_dict[key]['y']] for key in sorted(corners_dict.keys())])
                    corners = corners.flatten().reshape(-1, 2)
                    corners = corners.tolist()
                    corners = np.array(corners)
                    
                    # Function to reorder corners to form a rectangle
                    def reorder_corners(corners):
                        # Calculate the centroid
                        centroid = np.mean(corners, axis=0)
                            
                        # Calculate the angle of each point relative to the centroid
                        angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])
                            
                        # Sort the points by angle
                        sorted_indices = np.argsort(angles)
                        sorted_corners = corners[sorted_indices]
                            
                        return sorted_corners

                    # Reorder the corners
                    sorted_corners = reorder_corners(corners)

                    # Close the loop
                    perimeter = np.vstack([sorted_corners, sorted_corners[0]])

                    for i in range(len(perimeter) - 1):
                        start = perimeter[i]
                        end = perimeter[i + 1]
                        distance = np.linalg.norm(end - start)
                        num_points = int(distance)
                        for j in range(num_points + 1):
                            point = start + (end - start) * (j / num_points)
                            per_obstacles.append(point)

                
                
                if len(targets) == 0:
                    raise ValueError("No targets detected")

                # Convert to numpy arrays for further processing
                targets = np.array(targets)
                obstacles = np.array(obstacles)
                per_obstacles = np.array(per_obstacles)

                # Start and shift values
                start = np.array([0, 0])
                shift_x, shift_y = 500, 500  # Example shift values

                # Shift targets, obstacles, and start position
                shifted_targets = targets + [shift_x, shift_y]
                shifted_targets = shifted_targets.flatten().reshape(-1, 2)
                shifted_obstacles = obstacles + [shift_x, shift_y]
                shifted_obstacles = shifted_obstacles.flatten().reshape(-1, 2)
                shifted_per_obstacles = per_obstacles + [shift_x, shift_y]
                shifted_per_obstacles = shifted_per_obstacles.flatten().reshape(-1, 2)
                shifted_start = start + [shift_x, shift_y]
                shifted_start = shifted_start.flatten()

                # Make sure all obstacle coordinates are integers
                shifted_obstacles = shifted_obstacles.astype(int)
                shifted_per_obstacles = shifted_per_obstacles.astype(int)                

                # Create a grid and mark obstacles
                grid_size = (1000, 1000)
                grid = np.zeros(grid_size, dtype=bool)

                for obs in shifted_obstacles:
                    grid[obs[0], obs[1]] = True

                # Create a buffer around obstacles
                obstacles_buff = binary_dilation(grid, iterations=buff_size)

                # Dilate the perimeter obstacles
                perimeter_buff = np.zeros_like(obstacles_buff)
                for obs in shifted_per_obstacles:
                    perimeter_buff[obs[0], obs[1]] = True
                perimeter_buff = binary_dilation(perimeter_buff, iterations=2)

                # Combine perimeter with obstacles buffer
                obstacles_buff = np.logical_or(obstacles_buff, perimeter_buff)

                # Example usage
                start = tuple(shifted_start)
                target = tuple(shifted_targets[0])

                # Check if the start is within an obstacle buffer
                if obstacles_buff[start[0], start[1]]:
                    print("Start position is within an obstacle buffer")
                    plt.imshow(obstacles_buff.T, cmap='gray_r')
                    plt.scatter(shifted_start[0], shifted_start[1], c='blue', marker='o', label='Start')
                    plt.scatter(shifted_targets[:, 0], shifted_targets[:, 1], c='green', marker='x', label='Eva')
                    plt.legend()
                    plt.gca().invert_yaxis()  # Flip the y-axis
                    plt.show()
                    raise ValueError("Start position is within an obstacle buffer")
                    

                # Check if the target is within an obstacle buffer
                if obstacles_buff[target[0], target[1]]:
                    print("Target position is within an obstacle buffer")
                    raise ValueError("Target position is within an obstacle buffer")                

                path = a_star(obstacles_buff, start, target)
                print("Path:", path)

                # Simplify the path
                simplified_path = simplify_path(path, obstacles_buff)
                print("Simplified Path:", simplified_path)

                # Shift the path back to the original coordinates
                shifted_back_path = simplified_path - [shift_x, shift_y]
                print("Shifted Back Path:", shifted_back_path)                  

                # Plotting the path
                plt.imshow(obstacles_buff.T, cmap='gray_r')
                if simplified_path.size > 0:
                    simplified_path = np.array(simplified_path)
                    plt.plot(simplified_path[:, 0], simplified_path[:, 1], 'r-', linewidth=2)
                    plt.scatter(shifted_start[0], shifted_start[1], c='blue', marker='o', label='Start')
                    plt.scatter(shifted_targets[:, 0], shifted_targets[:, 1], c='green', marker='x', label='Eva')
                    plt.legend()
                    plt.show()

                for path_no in range(len(shifted_back_path) - 1):
                    
                    # Calculate the position state error
                    start_path_position = shifted_back_path[path_no + 1] # First path position is always 0,0 - skip this
                    start_target_position = shifted_back_path[-1]
                    
                    calculate_position_error_state_3(target_no, start_path_position, start_target_position)

                while status_dict['state3to4'] == 0:
                    time.sleep(0.1)  # Short sleep to avoid busy-waiting


                # Check if state has changed
                    if status_dict['state'] != 3:
                        break
                    time.sleep(1)  # Adjust the sleep time as needed
                    print("Waiting for target reached status")           

                # Switch to state 4
                status_dict['state'] = 4
                state_change_event.set()  # Signal the state change event
            
        elif status_dict['state'] == 4:

            status_dict['state'] = 4

            print("State 4: Executing state 4 logic")

            target_no = 0

            # Initialise error values
            client.publish('Wall-E/position_controller/error/dist', payload=None, qos=2, retain=True)
            client.publish('Wall-E/position_controller/error/theta', payload=None, qos=2, retain=True)

            # state 4 logic 
            while status_dict['state'] == 4:
                time.sleep(0.1)  # Short sleep to avoid busy-waiting

                # Extract targets and obstacles from dictionaries
                targets = []
                obstacles = []
                per_obstacles = []

                for key in sorted(plant_dict.keys()):
                    if plant_dict['x'] is not None and plant_dict['y'] is not None:
                        targets.append([int(plant_dict['x']), int(plant_dict['y'])])

                for key in sorted(obstacles_dict.keys()):
                    if obstacles_dict[key]['x'] is not None and obstacles_dict[key]['y'] is not None:
                        obstacles.append([int(obstacles_dict[key]['x']), int(obstacles_dict[key]['y'])])

                #for key in sorted(eva_dict.keys()):
                #    if eva_dict['x'] is not None and eva_dict['y'] is not None:
                #        obstacles.append([int(eva_dict['x']), int(eva_dict['y'])])

                for key in sorted(corners_dict.keys()):
                        if corners_dict[key]['x'] is not None and corners_dict[key]['y'] is not None:
                            per_obstacles.append([int(corners_dict[key]['x']), int(corners_dict[key]['y'])])

                if len(corners_dict) == 4:

                    # Convert the corners of the grid to a perimeter 
                    corners = np.array([[corners_dict[key]['x'], corners_dict[key]['y']] for key in sorted(corners_dict.keys())])
                    corners = corners.flatten().reshape(-1, 2)
                    corners = corners.tolist()
                    corners = np.array(corners)
                        
                    # Function to reorder corners to form a rectangle
                    def reorder_corners(corners):
                        # Calculate the centroid
                        centroid = np.mean(corners, axis=0)
                        
                        # Calculate the angle of each point relative to the centroid
                        angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])
                            
                        # Sort the points by angle
                        sorted_indices = np.argsort(angles)
                        sorted_corners = corners[sorted_indices]
                            
                        return sorted_corners

                    # Reorder the corners
                    sorted_corners = reorder_corners(corners)

                    # Close the loop
                    perimeter = np.vstack([sorted_corners, sorted_corners[0]])

                    for i in range(len(perimeter) - 1):
                        start = perimeter[i]
                        end = perimeter[i + 1]
                        distance = np.linalg.norm(end - start)
                        num_points = int(distance)
                        for j in range(num_points + 1):
                            point = start + (end - start) * (j / num_points)
                            per_obstacles.append(point)

                if len(targets) == 0:
                    print("No targets detected")
                    time.sleep(1)
                    break

                # Convert to numpy arrays for further processing
                targets = np.array(targets)
                obstacles = np.array(obstacles)
                per_obstacles = np.array(per_obstacles)

                # Start and shift values
                start = np.array([0, 0])
                shift_x, shift_y = 500, 500  # Example shift values

                # Shift targets, obstacles, and start position
                shifted_targets = targets + [shift_x, shift_y]
                shifted_targets = shifted_targets.flatten().reshape(-1, 2)
                shifted_obstacles = obstacles + [shift_x, shift_y]
                shifted_obstacles = shifted_obstacles.flatten().reshape(-1, 2)
                shifted_per_obstacles = per_obstacles + [shift_x, shift_y]
                shifted_per_obstacles = shifted_per_obstacles.flatten().reshape(-1, 2)
                shifted_start = start + [shift_x, shift_y]
                shifted_start = shifted_start.flatten()

                # Make sure all obstacle coordinates are integers
                shifted_obstacles = shifted_obstacles.astype(int)
                shifted_per_obstacles = shifted_per_obstacles.astype(int)

                # Create a grid and mark obstacles
                grid_size = (1000, 1000)
                grid = np.zeros(grid_size, dtype=bool)

                for obs in shifted_obstacles:
                    grid[obs[0], obs[1]] = True

                # Create a buffer around obstacles
                obstacles_buff = binary_dilation(grid, iterations=buff_size)

                # Dilate the perimeter obstacles
                perimeter_buff = np.zeros_like(obstacles_buff)
                for obs in shifted_per_obstacles:
                    perimeter_buff[obs[0], obs[1]] = True
                perimeter_buff = binary_dilation(perimeter_buff, iterations=2)

                # Combine perimeter with obstacles buffer
                obstacles_buff = np.logical_or(obstacles_buff, perimeter_buff)

                # Example usage
                start = tuple(shifted_start)
                target = tuple(shifted_targets[0])

                # Check if the start is within an obstacle buffer
                if obstacles_buff[start[0], start[1]]:
                    raise ValueError("Start position is within an obstacle buffer")

                path = a_star(obstacles_buff, start, target)
                print("Path:", path)

                # Simplify the path
                simplified_path = simplify_path(path, obstacles_buff)
                print("Simplified Path:", simplified_path)

                # Shift the path back to the original coordinates
                shifted_back_path = simplified_path - [shift_x, shift_y]
                print("Shifted Back Path:", shifted_back_path)               

                # Plotting the path
                plt.imshow(obstacles_buff.T, cmap='gray_r')
                if simplified_path.size > 0:
                    simplified_path = np.array(simplified_path)
                    plt.plot(simplified_path[:, 0], simplified_path[:, 1], 'r-', linewidth=2)
                plt.scatter(shifted_start[0], shifted_start[1], c='blue', marker='o', label='Start')
                plt.scatter(shifted_targets[:, 0], shifted_targets[:, 1], c='green', marker='x', label='Plant')
                plt.legend()
                plt.show()

                for path_no in range(len(shifted_back_path) - 1):
                        
                    # Calculate the position state error
                    start_path_position = shifted_back_path[path_no + 1] # First path position is always 0,0 - skip this
                    start_target_position = shifted_back_path[-1]
                    
                    calculate_position_error_state_4(target_no, start_path_position, start_target_position)

                # Send 'plant_reached' signal to event
                client.publish('Wall-E/plant_reached', 1) 

                # Switch to state 1 
                status_dict['state'] = 1          
                state_change_event.set()  # Signal the state change event
            
        else:
            print(f"Unknown state: {current_state}")
            time.sleep(1)

        # Wait for the state change event for subsequent state changes
        print("Waiting for state change event")
        state_change_event.wait()  # Wait for the state change event
        print(f"State change event received, current state: {status_dict['state']}")
        state_change_event.clear()  # Clear the event

        # Update the current state
        current_state = status_dict['state']

# Start the state machine in a separate thread
state_machine_thread = threading.Thread(target=state_machine)
state_machine_thread.start()
root.mainloop()






