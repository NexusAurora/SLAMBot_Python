
import numpy as np

#import pytorch
import torch
import torch.nn as nn
import torch.nn.functional as F

#! Callibration Model ------------------------------------------------------------------

class Linear(nn.Module):
    
    def __init__(self, input_dim):
        super(Linear, self).__init__()
        # Each element has its own weight for multiplication
        self.weights = nn.Parameter(torch.randn(input_dim))
        # Each element has its own bias for addition
        self.biases = nn.Parameter(torch.randn(input_dim))

    def forward(self, x):
        # Element-wise multiplication and addition
        return x * self.weights + self.biases

class Model(nn.Module):

    def __init__(self):
        
        super(Model, self).__init__()
        # Linear layer
        self.l1a = Linear(101)
        self.l1b = Linear(101)
        self.l1c = Linear(101)

        self.l6 = Linear(101)
        self.l7 = Linear(101)
        self.l8 = Linear(101)
        self.l9 = Linear(101)
        self.l10 = Linear(101)
        self.l11 = Linear(101)
        self.l12 = Linear(101)
        self.l13 = Linear(101)
        
        # Create a constant array (e.g., cosine values from 40 to 140 degrees)
        degrees = np.arange(40, 141)  # Create an array from 40 to 140
        radians = np.deg2rad(degrees)  # Convert degrees to radians
        cosine_values = np.cos(radians)  # Compute cosine
        
        # Convert the numpy array to a torch tensor and then to a Parameter, setting requires_grad to False
        self.cosine_constant = nn.Parameter(torch.tensor(cosine_values, dtype=torch.float32), requires_grad=False)

    def forward(self, x):

        #Each index is dependent on the previous and the next index 
        x_left_shift = torch.roll(x, 1, 1) 
        x_left_shift[:, -1] = 0

        x_right_shift = torch.roll(x, -1, 1)
        x_right_shift[:, 0] = 0

        x_pure_a = self.l1a(x)
        x_pure_b = self.l1b(x)

        x_front = self.l6(x_left_shift) + self.l7(x_right_shift) + x_pure_b
        x_front = F.gelu(self.l8(x_front))
        x_front = F.gelu(self.l9(x_front))
        x_front = self.l10(x_front)
        x_front = F.gelu(self.l11(x_front))
        x_front = self.l12(x_front)

        x_pure_c = self.l1c(x)
        x_angles = self.l13(self.cosine_constant*x_pure_c)

        return x + x_front + x_pure_a + x_angles

#load the model
model = Model().to('cuda')
model.load_state_dict(torch.load('calibration_model.pth'))

#! Coordinate Conversion ----------------------------------------------------------------
#? Converts (distance, angle) to (x, y) coordinates ----------------------------------------------

def plot(data, front_start, front_end, num_readings, fix_error = 0, servo_offset = 2.8, body_offset_y = 8, 
        min_distance = 3, max_distance = 100):

    """
    Plot the coordinates of the detected objects based on the given data.

    Args:
        data (dict): A dictionary containing the sensor readings.
        front_start (float): The starting angle of the front sensor.
        front_end (float): The ending angle of the front sensor.
        back_start (float): The starting angle of the back sensor.
        back_end (float): The ending angle of the back sensor.
        num_readings (int): The number of sensor readings.
        fix_error (float, optional): The error to be fixed. Defaults to 0.
        servo_offset (float, optional): The offset of the servo. Defaults to 2.8.
        body_offset_y (float, optional): The offset of the body in the y-axis. Defaults to -8.
        min_distance (float, optional): The minimum distance threshold. Defaults to 3.
        max_distance (float, optional): The maximum distance threshold. Defaults to 100.

    Returns:
        zip: A zip object containing the x and y coordinates of the detected objects.
    """

    #! Front - 40 to 140 --------------------------------------------------
    front = np.array(data['f'])
    
    #convert to tensor
    front = torch.tensor(front, dtype=torch.float32).to('cuda')
    front = front.unsqueeze(0)

    #predict
    #* Deep learning model to calibrate, filter and adjust incoming scan values
    front = model(front)
    front = front.cpu().detach().numpy()
    front = front[0]

    print(front)

    #convert to cm
    front = front/10

    #*Fix servo offset and error
    front = front + servo_offset + fix_error
    front_min_distance  = min_distance + servo_offset + fix_error
    front_max_distance  = max_distance + servo_offset + fix_error

    #[40,41,42,43,44,45................. 140]
    angles = np.linspace(np.deg2rad(front_start), np.deg2rad(front_end), num_readings)

    #delete data from both if front is 0
    for i in range(len(angles) - 1, -1, -1):
        if front[i] < front_min_distance or front[i] > front_max_distance:
            angles = np.delete(angles, i)
            front = np.delete(front, i)

    #* Converting Servo measurement into actual distance using trigonometry
    x_coords = -1 * front * np.cos(angles)
    y_coords = front * np.sin(angles) 

    #*Fix body offset and error
    #fix the y offset
    y_coords = y_coords + body_offset_y

    # [x1, x2, x3, ........ ] [y1, y2, y3, ........ ] ----> [(x1, y1), (x2, y2), (x3, y3), ........ ]
    return zip(x_coords, y_coords)

#! Occupancy Grid Mapping --------------------------------------------------------------

#? Probabilistic Occupancy Grid Mapping (POGM) With Temporal Filtering ----------------------------
def update_occupancy_grid(coordinates, map_height, map_width, CELL_SIZE, occupancy_grids, alpha=0.7, coordinate_filter_percent=0.1, coordinate_move_percent=0.25, clump_decay=0.05):
    
    """
    Update the occupancy grid map based on the given coordinates.

    Parameters:
    - coordinates: A list of tuples, where each tuple represents the x and y coordinates.
    - map_height: The height of the map.
    - map_width: The width of the map.
    - CELL_SIZE: The size of each cell in the grid.

    Returns:
    - occupancy_grid: An updated occupancy grid map.
    """

    filtered_coordinates = []
    updated_coordinate_indexes = []
    updated_coordinate_values = []
    updated_grid_indexes = []

    #* Temporal filter to reduce noise ( on past occupancy grids ) ------------------------
    # Avarage the last N occupancy grids to reduce noise (latest map has the highest weight) - weigted average
    if len(occupancy_grids) > 0:
        # Generate increasing weights for each occupancy grid
        weights = np.arange(1, len(occupancy_grids) + 1)
        # Apply weights to each grid
        weighted_grids = [occupancy_grids[i] * weights[i] for i in range(len(occupancy_grids))]
        # Calculate the weighted sum of grids
        weighted_sum = np.sum(weighted_grids, axis=0)
        # Normalize by the sum of weights to get the weighted average
        occupancy_grid = weighted_sum / np.sum(weights)
    else:
        occupancy_grid = np.zeros((map_height, map_width))

    #*Plotting probabilistic occupancy grid map ------------------------------------------
    for x, y in coordinates:
        # Calculate grid indices
        grid_x = int((x // CELL_SIZE) + map_width / 2)
        grid_y = int((y // CELL_SIZE) + map_height / 2)

        '''# Check if the indices are within the grid bounds
        if 0 <= grid_x < map_width and 0 <= grid_y < map_height:
            # Mark the cell as occupied
            occupancy_grid[grid_y, grid_x] = alpha * occupancy_grid[grid_y, grid_x] + (1 - alpha) * 1'''
        
        # Check if the indices are within the grid bounds
        if 0 <= grid_x < map_height and 0 <= grid_y < map_width:
            # Mark the cell as occupied
            occupancy_grid[grid_x, grid_y] = alpha * occupancy_grid[grid_x, grid_y] + (1 - alpha) * 1

            updated_coordinate_indexes.append((x, y))
            updated_coordinate_values.append(occupancy_grid[grid_x, grid_y])
            updated_grid_indexes.append((grid_x, grid_y))

    #* Filter coordinates based on the percentage of the grid that has been updated ---------
    min_value = np.min(updated_coordinate_values)
    max_value = np.max(updated_coordinate_values)
    threshold = min_value + (max_value - min_value) * coordinate_filter_percent

    print("min and max value in occ_grid: ", min_value, max_value)
    print("threshold calculated: ", threshold)

    for i in range(len(updated_coordinate_indexes)):

        x, y = updated_grid_indexes[i]

        is_considred = False
        #filter coordinates based on threshold
        if updated_coordinate_values[i] > threshold:
            filtered_coordinates.append((updated_coordinate_indexes[i][0], updated_coordinate_indexes[i][1]))
            is_considred = True

        # Decay surrounding cells (if index not in updated_coordinate_indexes) [of occupancy grid]

        max_surrounding_prob = 0
        surrounding_prob_index = None
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = int(x + dx), int(y + dy)
                if 0 <= nx < occupancy_grid.shape[0] and 0 <= ny < occupancy_grid.shape[1]:

                    if (nx, ny) not in updated_grid_indexes:
                        occupancy_grid[nx, ny] = occupancy_grid[nx, ny] * (1 - clump_decay)
                    
                    if not is_considred:
                        if occupancy_grid[nx, ny] > max_surrounding_prob:
                            max_surrounding_prob = occupancy_grid[nx, ny]
                            surrounding_prob_index = (nx, ny)

        if not is_considred and surrounding_prob_index is not None and max_surrounding_prob > threshold:
            #find out direction w.r.t x, y
            x_diff = surrounding_prob_index[0] - x
            y_diff = surrounding_prob_index[1] - y

            #move towards the direction
            filtered_coordinates.append((updated_coordinate_indexes[i][0] + (x_diff*coordinate_move_percent), updated_coordinate_indexes[i][1] + (y_diff*coordinate_move_percent) ))

    return occupancy_grid, filtered_coordinates

#? Find points between two coordinates -------------------------------------------------

def bresenham(x0, y0, x1, y1):
    """
    Bresenham's Line Algorithm to generate points between two coordinates.
    """
    
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

#? Update Free Space Grid --------------------------------------------------------------

def update_free_space_grid(robot_pose, coordinates, map_height, map_width, CELL_SIZE, free_space_grids, alpha=0.7):
    
    #* Temporal filter to reduce noise ( on past Free Space grids ) ------------------------
    if len(free_space_grids) > 0:
        weights = np.arange(1, len(free_space_grids) + 1)
        weighted_grids = [free_space_grids[i] * weights[i] for i in range(len(free_space_grids))]
        weighted_sum = np.sum(weighted_grids, axis=0)
        free_space_grid = weighted_sum / np.sum(weights)
    else:
        free_space_grid = np.zeros((map_height, map_width))
    
    #* Plotting robot's current position ( cell ) and rotation -----------------------------
    # Get the robot's position from the homogeneous transformation matrix (3x3)
    #if not numpy array convert
    if not isinstance(robot_pose, np.ndarray):
        robot_pose = np.array(robot_pose)
    robot_x, robot_y = robot_pose[:2, 2]

    #robot rotation
    robot_rotation = np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
    #Convert to degrees
    robot_rotation = np.rad2deg(robot_rotation)

    # Calculate grid indices for the robot
    robot_grid_x = int((robot_x // CELL_SIZE) + map_width / 2)
    robot_grid_y = int((robot_y // CELL_SIZE) + map_height / 2)

    #* Plotting wall ( obstacles ) ---------------------------------------------------------
    for x, y in coordinates:
        # Calculate grid indices for the wall
        grid_x = int((x // CELL_SIZE) + map_width / 2)
        grid_y = int((y // CELL_SIZE) + map_height / 2)

        #* Finding free space cells using bresenham algorithm --------------------------------
        # Get the points along the ray from the robot to the wall
        points = bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

        '''for px, py in points:
            if 0 <= px < map_width and 0 <= py < map_height:
                free_space_grid[py, px] = alpha * free_space_grid[py, px] + (1 - alpha) * 1'''

        #* Plotting probabilistic free space grid map ---------------------------------------
        for px, py in points:
            if 0 <= px < map_height and 0 <= py < map_width:
                free_space_grid[px, py] = alpha * free_space_grid[px, py] + (1 - alpha) * 1

    #check if robot is in the grid, if not return None
    if robot_grid_x < 0 or robot_grid_x >= map_height or robot_grid_y < 0 or robot_grid_y >= map_width:
        robot_cell = None
    else:
        robot_cell = (robot_grid_x, robot_grid_y)    

    return free_space_grid, robot_cell, robot_rotation

#! Helper Tramsformation Functions --------------------------------------------------------------

def transform_points(T, A):
    return np.dot(T, np.concatenate((A.T, np.ones((1, A.shape[0]))), axis=0)).T[:, :2]

def transform_points_inverse(T, A):
    T_inv = np.linalg.inv(T)
    return np.dot(T_inv, np.concatenate((A.T, np.ones((1, A.shape[0]))), axis=0)).T[:, :2]

def transform_matrix(T, A):
    return np.dot(T, A)

def transform_matrix_inverse(T, A):
    T_inv = np.linalg.inv(T)
    return np.dot(T_inv, A)

def homogeneous_inbetween(T1, T2, alpha):
    return np.dot(T1, np.linalg.matrix_power(np.dot(np.linalg.inv(T1), T2), alpha))