# Thesis: A probabilistic learning-based routing scheme with path planning of autonomous agricultural vehicles for assisted harvesting tasks

## General Description

This work introduces a combined route and path planning strategy to guide autonomous ground vehicles in scheduled
harvesting tasks within expansive crop rows subject to complex terrain conditions of agricultural fields. The proposed
planning strategy integrates:

1. A global planning method based on the Traveling Salesman Problem under the Capacitated Vehicle Routing approach to
   prioritize harvesting positions according to criteria of minimum path length and vehicle's load capacity.

2. A local planning strategy based on an Informed Rapidly-exploring Random Trees (RRT*) to coordinate the previously
   scheduled harvesting points while simultaneously avoiding obstacles of low-traction terrain.

Moreover, the global approach aims to generate an ordered queue of harvesting locations, maximizing the harvested
product of the expected crop yield. On the other hand, in the second stage, the informed RRT* planner avoids obstacles
around the configuration space of the robotic vehicle and incorporates a dynamical model of the vehicle motion dynamics
to meet constraints compatible with those of the planned path. Experimental results demonstrated the effectiveness of
the proposed planning strategy, achieving smooth and collision-free paths, making it suitable for practical applications
in precision agriculture.

## Prerequisites

Make sure you have the following installed before starting:

- CoppeliaSim (Version 4.5.1)
- Python (Version 3.9)

## Installation

1. **CoppeliaSim:**
    - Download CoppeliaSim from [the official website](https://www.coppeliarobotics.com/downloads).
    - Follow the installation instructions provided in the documentation.

2. **Python and Dependencies:**
    - Install Python > 3.9 from [python.org](https://www.python.org/downloads/).
    - Install the required dependencies by running the following command:
       ```bash
       pip install -r requirements.txt
       ```

## Obtaining the CoppeliaSim Map

1. Open CoppeliaSim
1. Open the file from the coppelia_map folder

## Considerations

- The map must have a working plane of 1x15 meters.
- If you want to run the application On Line, obtaining the map from Coppelia, you must change the value of the variable
  ON_LINE to True in the config.py file.
- If you want to run the application Off Line, obtaining the map from a file, you must change the value of the variable
  ON_LINE to False in the config.py file.

## Running the Program

1. Use the `app.py` script from the steps folder to execute the program.
    ```bash
    python app.py
    ```

## Running in Parts

1. Use the `step_1_get_data.py` script from the steps folder to obtain the processed map from Coppelia.
    ```bash
    python step_1_get_data.py
    ```

2. Use the `step_2_tsp.py` script from the steps folder to execute the TSP algorithm.
    ```bash
    python step_2_tsp.py
    ```

3. Use the `step_3_rrt.py` script from the steps folder to execute the RRT algorithm.
    ```bash
    python step_3_rrt.py
    ```
