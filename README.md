# SLAMBot Python

## Overview

SLAMBot Python is a robotic system designed to navigate and map its environment using various sensors and algorithms. This project implements functionalities such as movement control, particle filtering, pathfinding, and data visualization, all written in Python.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Directory Structure](#directory-structure)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Sensor Calibration**: Includes tools for calibrating LiDAR and movement sensors.
- **Movement Control**: Implements movement algorithms for precise navigation.
- **Particle Filtering**: Utilizes particle filters for localization and mapping.
- **Pathfinding**: Implements algorithms for efficient navigation through environments.
- **Data Visualization**: Provides plotting tools to visualize sensor data and maps.

## Installation

To set up the SLAMBot Python project, follow these steps:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/NexusAurora/SLAMBot_Python.git
   cd SLAMBot_Python
   ```

2. **Install Dependencies**:
   Make sure you have Python installed, then install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the Main Script**:
   You can start the main program by running:
   ```bash
   python main.py
   ```

## Usage

### Calibration

- **LiDAR Calibration**: Use the Jupyter Notebook `lidar_calibration.ipynb` to calibrate the LiDAR sensor.
- **Movement Calibration**: Use the Jupyter Notebook `movement_calibration.ipynb` to calibrate movement parameters.

### Running the Algorithms

- **Movement Control**: The `movement.py` script contains functions to control the robot's movement.
- **Particle Filtering**: The `particle_filter.py` script implements the particle filtering algorithm for localization.
- **Pathfinding**: The `path_finding.py` script contains the pathfinding algorithms used for navigation.
- **Data Visualization**: Use `plotter.py` and `plotting_map.py` to visualize the data collected from the sensors.

## Directory Structure

```
SLAMBot_Python/
│   calibration_model.pth
│   lidar_calibration.ipynb
│   lidar_calibration_data.csv
│   main.py
│   movement.py
│   movement_calibration.ipynb
│   movement_weights.npy
│   openCV_display.py
│   particle_filter.py
│   particle_filter_manual.py
│   path_finding.py
│   plotter.py
│   plotting_map.py
│   test.ipynb
│   test.py
│
├───Checks
│       BareMinimum.txt
│       Checks.ino
│
├───FastLider
│       FastLider.ino
│
├───Lider
│       Lider.ino
│
├───LN298N
│       LN298N.ino
│
├───Rader
│   └───Rader
│           Rader.ino
│
├───Servo
│   └───Servo
│           Servo.ino
│
├───TOF
│       TOF.ino
│
├───Ultrasonic
│       Ultrasonic.ino
│
└───__pycache__
        movement.cpython-311.pyc
        openCV_display.cpython-311.pyc
        particle_filter.cpython-311.pyc
        particle_filter_manual.cpython-311.pyc
        path_finding.cpython-311.pyc
        plotter.cpython-311.pyc
```

## Contributing

Contributions are welcome! If you have suggestions for improvements or new features, please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

