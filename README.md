# An Integrated Route and Path Planning Strategy for Skid--Steer Mobile Robots in Assisted Harvesting Tasks with Terrain Traversability Constraints

## General Description

This article presents a combined route and path planning strategy to guide Skid–Steer Mobile Robots (SSMRs) in scheduled
harvest tasks within expansive crop rows with complex terrain conditions. The proposed strategy integrates: (i) a global
planning algorithm based on the Traveling Salesman Problem under the Capacitated Vehicle Routing approach and
Optimization Routing (OR-tools from Google) to prioritize harvesting positions by minimum path length, unexplored
harvest points, and vehicle payload capacity; and (ii) a local planning strategy using Informed Rapidly-exploring Random
Tree (IRRT∗) to coordinate scheduled harvesting points while avoiding low-traction terrain obstacles. The global
approach generates an ordered queue of harvesting locations, maximizing the crop yield in a workspace map. In the second
stage, the IRRT∗ planner avoids potential obstacles, including farm layout and slippery terrain. The path planning
scheme incorporates a traversability model and a motion model of SSMRs to meet kinematic constraints. Experimental
results in a generic fruit orchard demonstrate the effectiveness of the proposed strategy. In particular, the IRRT∗
algorithm outperformed RRT and RRT∗ with 96.1% and 97.6% smoother paths, respectively. The IRRT∗ also showed improved
navigation efficiency, avoiding obstacles and slippage zones, making it suitable for precision agriculture.

## Prerequisites

Make sure you have the following installed before starting:

- CoppeliaSim (Version 4.5.1)
- Python (Version 3.9)

## Installation

1. **CoppeliaSim:**
    - Download CoppeliaSim from [the official site](https://www.coppeliarobotics.com/downloads).
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

- The map must have a working plane of 1x15 meters
- If you want to run the application On Line getting the map from coppelia, you must change in the config.py file
  the value of the ON_LINE variable to True
- If you want to run the application Off Line getting the map from a file, you must change in the config.py file
  the value of the ON_LINE variable to False

## Program Execution

1. Use the `app.py` script from the steps folder to run the program
    ```bash
    python app.py
    ```

## Execution by Parts

1. Use the `step_1_get_data.py` script from the steps folder to obtain the processed map from coppelia
    ```bash
    python step_1_get_data.py

2. Use the `step_2_tsp.py` script from the steps folder to run the TSP algorithm
    ```bash
    python step_2_tsp.py

3. Use the `step_3_rrt.py` script from the steps folder to run the RRT algorithm
    ```bash
    python step_3_rrt.py

## Notes

This project was carried out at the Universidad Católica del Norte of Antofagasta in the Department of Systems and
Computer Engineering (DISC).

If you wish to use this code in your work, please cite our paper available
at [this link](https://www.preprints.org/manuscript/202406.0326/v1).