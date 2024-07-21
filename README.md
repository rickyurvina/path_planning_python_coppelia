# An Integrated Route and Path Planning Strategy for Skid--Steer Mobile Robots in Assisted Harvesting Tasks with Terrain Traversability Constraints

## Overview

This project introduces a combined route and path planning strategy for guiding autonomous ground vehicles during
scheduled harvesting tasks. The strategy is designed to handle expansive crop rows and complex terrain conditions of
agricultural fields. The proposed planning strategy integrates:

1. A global planning method based on the Traveling Salesman Problem (TSP) under the Capacitated Vehicle Routing
   approach. This method prioritizes harvesting positions based on criteria such as minimum path length and vehicle's
   load capacity.

2. A local planning strategy based on Informed Rapidly-exploring Random Trees (IRRT*). This strategy coordinates the
   previously scheduled harvesting points while simultaneously avoiding obstacles of low-traction terrain.

The global approach aims to generate an ordered queue of harvesting locations, maximizing the harvested product of the
expected crop yield. In the second stage, the informed RRT* planner avoids obstacles around the configuration space of
the robotic vehicle. It incorporates a dynamical model of the vehicle motion dynamics to meet constraints compatible
with those of the planned path. Experimental results demonstrate the effectiveness of the proposed planning strategy,
achieving smooth and collision-free paths, making it suitable for practical applications in precision agriculture.

## Getting Started

Clone the repository using the following command:

```bash
git clone https://github.com/rickyurvina/path_planning_python_coppelia.git
```

## Prerequisites

Ensure the following are installed before starting:

- CoppeliaSim (Version 4.5.1)
- Python (Version 3.9)
- MySql (Version 8.0.26)

## Installation

1. **CoppeliaSim:**
    - Download CoppeliaSim from the [official website](https://www.coppeliarobotics.com/downloads).
    - Follow the installation instructions provided in the documentation.

2. **Python and Dependencies:**
    - Install Python > 3.9 from [python.org](https://www.python.org/downloads/).
    - Install the required dependencies by running the following command:
       ```bash
       pip3 install -r requirements.txt
       ```

3. **MySql:**
    - Install MySql from the [official website](https://dev.mysql.com/downloads/mysql/).
    - Follow the installation instructions provided in the documentation.

## CoppeliaSim Map

1. Open CoppeliaSim.
2. Open the file `environment_harvesting_test.ttt` from the `coppelia_map` folder.

## Considerations

- The map must have a working plane of 1x15 meters.
- If you want to run the application online, obtaining the map from Coppelia, you must change the value of the `ON_LINE`
  variable to 1 in the `config.py` file.
- If you want to run the application offline, obtaining the map from a file, you must change the value of the `ON_LINE`
  variable to 0 in the `config.py` file.

## Python Environment Configuration

1. Create a virtual environment:
    ```bash
    python3 -m venv venv
    ```
2. Activate the virtual environment:
    ```bash
    source venv/bin/activate
    ```
3. Install the required dependencies:
    ```bash
    pip3 install -r requirements.txt
    ```
4. Deactivate the virtual environment:
   ```bash
   deactivate
   ```

## Database Creation

1. Update the MySql variables in the `src/steps/config.py` file according to your MySql configuration.
2. Use the `create_database.py` script from the `steps` folder to create the database:
    ```bash
    python3 src/steps/create_database.py
    ```

## Solutions Folder

1. Create a folder named `solutions` in the root of the project. This folder will store the solutions of the TSP and RRT
   algorithms.
2. Move the `solution_2049_07052024` folder to the `solutions` folder. If you generate new solutions, you can update
   the `SOLUTION` constant in the `config.py` file to point to the new folder.

## Running the Program

Use the `app.py` script from the `steps` folder to execute the program:

```bash
python3 src/steps/app.py
```

## Running in Parts

1. Use the `step_1_get_data.py` script from the `steps` folder to obtain the processed map from Coppelia. Ensure that
   the `ON_LINE` variable in `steps/config.py` is set to 1, and that the `connectionPort`
   in `components/step1_get_data.py` matches your Coppelia port. Load the map located
   in `coppelia_map/environment_harvestng_test.ttt` in CoppeliaSim. The simulation in Coppelia starts automatically:
    ```bash
    python3 src/steps/step_1_get_data.py
    ```

2. Use the `step_2_tsp.py` script from the `steps` folder to execute the TSP algorithm. You can run this file online or
   offline. If running online, ensure Coppelia is open with the map loaded. If running offline, set the `ON_LINE`
   variable in `config.py` to 0. If you want to run the TSP step offline, ensure that the `solutions` folder contains
   the `app_variables1.pickle` file. In `config.py`, you can adjust the `VEHICLE_CAPACITIES` and `START_POINT_TSP`
   values:
    ```bash
    python src/steps/step_2_tsp.py
    ```

3. Use the `step_3_rrt.py` script from the `steps` folder to execute the RRT algorithm. You can run this file online or
   offline. If running online, ensure the `ON_LINE` variable in `config.py` is set to 1 and that Coppelia is running
   with the map open. If running offline, set the `ON_LINE` variable in `config.py` to 0. This will load the solution
   from the `solutions` folder, using the `SOLUTION` constant value from `config.py`. If running offline, the path will
   be generated with the TSP Solution saved in the `solutions` folder. In the main function of `step_3_rrt.py`, you can
   uncomment the line of the different RRT methods for comparison. In `config.py`, you can adjust various parameters:
    ```bash
    python src/steps/step_3_rrt.py
    ```

## Notes

This project was carried out at the Universidad Católica del Norte of Antofagasta in the Department of Systems and
Computer Engineering (DISC).

If you wish to use this code in your work, please cite our paper available
at [this link](https://www.preprints.org/manuscript/202406.0326/v1).
