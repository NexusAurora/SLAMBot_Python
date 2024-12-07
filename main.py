import socket
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2
from plotter import plot, transform_points, transform_points_inverse, transform_matrix, update_occupancy_grid, update_free_space_grid
from particle_filter import icp
from openCV_display import process_frame, process_frame_path
from movement import generate_movement_commands
from path_finding import astar
import threading
import keyboard
import time

#? Variables -----------------------
#! Grid ---------------------------

# Constants
FOV_DEGREES = 101
NUM_READINGS = 101
CELL_SIZE = 2
WINDOW_SIZE = 5

# Initialize an empty occupancy grid map
map_width = 107//(CELL_SIZE - 1)
map_height = 107//(CELL_SIZE - 1)
occupancy_grids = [np.zeros((map_height, map_width))]
free_space_grids = [np.zeros((map_height, map_width))]
path_grid = np.zeros((map_height, map_width))

probability_filter = 0

#ICP
icp_iterations = 2
icp_tolerance = 0.0001

#display
display_scale = 6
guide_color = (225, 86, 43)

#! Self port ----------------------

localIP = socket.gethostbyname(socket.gethostname())
localPort = 1234

#! ESP32 port ----------------------

espIP = None
espPort = 1234 

bufferSize = 2024

print(f"Own Local IP Address: {localIP}")

# Create a datagram socket
serverSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
serverSocket.bind(("", localPort))

# don't block the socket
serverSocket.setblocking(False)

print(f"UDP server up and listening on port {localPort}")

#! Scans ---------------------------

previous_coordinates = []
coordinates = None

init_pose = None
robot_pose = (0, 0, 0)  # (x, y, theta)
robot_pose = np.array([
    [np.cos(robot_pose[2]), -np.sin(robot_pose[2]), robot_pose[0]],
    [np.sin(robot_pose[2]),  np.cos(robot_pose[2]), robot_pose[1]],
    [0,              0,             1]
])
robot_cell = (0, 0)
robot_rotation = 0

end_coordinate = (0, 30)
end_cell = (0, 0)

#! Path ----------------------------

path = None
simplified_path = None

#! Movement ------------------------

wait_time = 5  #seconds - waiting for the robot to reach the target
wait_to_send = 0.5 #wait time to send data

#? Functions & Classes --------------
#! Threads --------------------------

class UpdateGrids(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):

        global previous_coordinates
        global coordinates
        global robot_pose
        global occupancy_grids
        global free_space_grids
        global probability_filter
        global icp_iterations
        global icp_tolerance
        global FOV_DEGREES
        global NUM_READINGS
        global map_height
        global map_width
        global CELL_SIZE
        global WINDOW_SIZE
        global espIP
        global espPort
        global serverSocket
        global bufferSize
        global robot_cell
        global robot_rotation
        global init_pose

        while True:

            #? Receive data from the ESP32 --------------------------------------

            try:

                #* Receiving data coming through UDP
                message, address = serverSocket.recvfrom(bufferSize)
                message = message.decode('utf-8')  # Decode message to string
                print(address)

                if espIP is None:
                    espIP = address[0]
                    print(f"ESP32 IP Address: {espIP}")

                #* Parsing Json
                data = json.loads(message)
                #print in green color
                print("\033[92m", "Data: ", data, "\033[0m")
                
                '''
                #store in a csv file data, 25
                with open('lidar_calibration_data.csv', 'a') as f:
                    f.write(f"{data}, {10.4}\n")'''

                #! Calculations ---------------------------------------------------

                #* Calculate (x, y) coordinates of the LiDAR points ---------------
                coordinates = plot(data, 40, 140, NUM_READINGS) # -----> [(x1, y1), (x2, y2), ... ]

                #* Mapping & Particle Filtring ------------------------------------
                if(previous_coordinates):

                    #! ICP ---------------------------------------------------------
                    #to numpy array

                    #* Considering history to reduce noise and errors
                    A = np.concatenate(previous_coordinates[-WINDOW_SIZE:])
                    B = np.array(list(coordinates))
                    
                    #print in blue color
                    print("\033[94m", "Coordinates: ", B, "\033[0m")
                    
                    #check if B is empty
                    if B.size == 0:
                        print("Input arrays A and B must not be empty.")
                        continue
                    
                    if init_pose is not None:
                        #calculate initial pose w.r.t the robot
                        init_pose = np.dot(robot_pose, init_pose)
                        #* ICP ( Particle filtering)
                        T_final, distances, iterations = icp(A, B, init_pose=init_pose, max_iterations=icp_iterations, tolerance=icp_tolerance)
                        init_pose = None
                    else:
                        #* ICP ( Particle filtering)
                        T_final, distances, iterations = icp(A, B, max_iterations=icp_iterations, tolerance=icp_tolerance)
                    
                    #inverse of T
                    T_inv = np.linalg.inv(T_final)

                    #New coordinates
                    #* Fix Robot pose and new coordinates using the transformation matrix
                    # Fixing coordinates
                    B = transform_points_inverse(T_final, B)
                    coordinates = B

                    #new robot pose
                    robot_pose = T_inv
                
                #* Storing Each Measurement History -------------------------------
                # Update past coordinates
                # Window size determines how many previous measurements to consider
                previous_coordinates.append(np.array(list(coordinates)))
                if len(previous_coordinates) > WINDOW_SIZE:
                    previous_coordinates.pop(0)
                
                #* Update the occupancy grid map ----------------------------------
                occupancy_grid, _ = update_occupancy_grid(coordinates, map_height, map_width, CELL_SIZE, occupancy_grids, alpha=0.9)

                #* Storing the calculated occupancy grid in history ---------------
                # Update past occupancy grids
                occupancy_grids.append(occupancy_grid)
                if(len(occupancy_grids) > WINDOW_SIZE):
                    occupancy_grids.pop(0)

                #* Update the free space grid map ----------------------------------
                free_space_grid, robot_cell, robot_rotation = update_free_space_grid(robot_pose, coordinates, map_height, map_width, CELL_SIZE, free_space_grids, alpha=0.9)

                print("\nCurrent Robot Cell: ", robot_cell)
                print("Current Robot Rotation: ", robot_rotation , "\n")

                #* Storing the calculated free space grid in history ---------------
                # Update past free space grids
                free_space_grids.append(free_space_grid)
                if(len(free_space_grids) > WINDOW_SIZE):
                    free_space_grids.pop(0)

                #filter out cells with low probability
                occupancy_grid[occupancy_grid < probability_filter] = 0
                free_space_grid[free_space_grid < probability_filter] = 0

            except BlockingIOError:
                # No data is available, skip
                pass

class Display(threading.Thread):
    
        def __init__(self):
            threading.Thread.__init__(self)

        def run(self):
    
            global occupancy_grids
            global free_space_grids
            global display_scale
            global guide_color
            global robot_cell
            global robot_rotation
            global end_cell
            global path
            global simplified_path

            while True:

                #! Display ---------------------------------------------------------------

                if(len(occupancy_grids) > 0):

                    occupancy_grid = occupancy_grids[-1]
                    free_space_grid = free_space_grids[-1]

                    processed_occupancy_grid = process_frame(occupancy_grid, cv2.COLORMAP_JET, robot_cell, robot_rotation, display_scale, guide_color)
                    processed_free_space_grid = process_frame(free_space_grid, cv2.COLOR_GRAY2BGR, robot_cell, robot_rotation, display_scale, guide_color)
                    processed_path_grid = process_frame_path(free_space_grid, cv2.COLOR_BGR2GRAY, robot_cell, robot_rotation, end_cell, simplified_path, display_scale, guide_color)
                    
                    #* Display ---------------------------------------------------------------
                    # Display the occupancy grid map
                    cv2.imshow('Occupancy Grid Map', processed_occupancy_grid)
                    cv2.imshow('Free Space Grid Map', processed_free_space_grid)
                    cv2.imshow('Path Grid Map', processed_path_grid)

                    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
                        cv2.destroyAllWindows()  # Close all OpenCV windows
                        break  # Exit the loop

class SendData(threading.Thread):
    
        def __init__(self):
            threading.Thread.__init__(self)
    
        def run(self):
    
            global espIP
            global espPort
            global serverSocket
    
            while True:

                if keyboard.is_pressed('s'):

                    print("Key 's' is pressed, entering manual mode")

                    if espIP is not None:

                        to_send = input("Enter the data to send: ")
                        # [f,l,r,b] -> [front, back, left, right, stop] -> [0,1,2,3,4] followed by number (int)
                        to_send = to_send.split()
                        payload = {
                            "d": int(to_send[0]), #direction
                            "t": int(to_send[1]), #time
                            "s": int(to_send[2]), #speed
                        }
                        payload = json.dumps(payload)
                        print(f"Sending data to ESP32: {payload}")
                        serverSocket.sendto(payload.encode('utf-8'), (espIP, espPort))

class PathFinder(threading.Thread):
        
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):

        global robot_cell
        global robot_rotation
        global occupancy_grids
        global free_space_grids
        global map_height
        global map_width
        global CELL_SIZE
        global WINDOW_SIZE
        global probability_filter
        global end_coordinate
        global end_cell
        global path
        global simplified_path

        while True:

            #* Calculating goal position (cell) in free space grid
            end_x, end_y = end_coordinate
            # Calculate grid indices for the end
            end_cell = (int(round(end_x / CELL_SIZE) + map_width / 2), int(round(end_y / CELL_SIZE) + map_height / 2))
            
            path, simplified_path = astar(robot_cell, end_cell, free_space_grids[-1], map_height, map_width)
            
    def send_data(self, payload):
        if espIP is not None:
            payload = json.dumps(payload)
            print(f"Sending data to ESP32: {payload}")
            serverSocket.sendto(payload.encode('utf-8'), (espIP, espPort))

class Movement(threading.Thread):
        
        def __init__(self):
            threading.Thread.__init__(self)
        
        def send_data(self, payload):
            if espIP is not None:
                payload = json.dumps(payload)
                print(f"Sending data to ESP32: {payload}")
                serverSocket.sendto(payload.encode('utf-8'), (espIP, espPort))

        def run(self):

            global simplified_path
            global wait_time
            global robot_rotation
            global init_pose
            global wait_time

            while True:
                if simplified_path:
                    command, approx_pose = generate_movement_commands(simplified_path, robot_rotation)
                    if command:
                        #print in yellow color
                        print("\033[93m", "Command: ", command, "\033[0m")
                        self.send_data(command)
                        time.sleep(wait_time)
                        init_pose = approx_pose
                        time.sleep(command['t']/1000+wait_time)

#? Threads, Creating Objects ---------------------------

update_grid_thread = UpdateGrids()
display_thread = Display()
send_data_thread = SendData()
path_finder_thread = PathFinder()
movement_thread = Movement()

#? Starting Threads -------------------------------------

print("\033[91mStarting threads --------------------------------------------\033[0m")
print("\033[91mStarting 'update_grid_thread' *******************************\033[0m")

#! Starting thread - that updates the grids
update_grid_thread.start()

print("\033[91mStarting 'display_thread' **********************************\033[0m")

#! Starting thread - that displays the grids
display_thread.start()

print("\033[91mStarting 'send_data_thread' ********************************\033[0m")

#! Starting thread - that sends data to the ESP32 (This is for manual control, Not needed for autonomous control)
send_data_thread.start()

print("\033[91mStarting 'path_finder_thread' ******************************\033[0m")

#! Starting thread - that finds the path
path_finder_thread.start()

time.sleep(30)

print("\033[91mStarting 'movement_thread' *********************************\033[0m")

#! Starting thread - that moves the robot
movement_thread.start()