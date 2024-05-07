import traceback

import numpy as np
from colorama import init, Fore
from matplotlib import pyplot as plt
from src.components.step_3_rrt.informed_rrt import RRT  # Importing Informed RRT class
from src.components.step_3_rrt.shift_positions import shift_positions  # Importing function to shift positions
from src.components.common import load_files, save_files  # Importing common functions for loading and saving files
from src.components import create_folder  # Importing function to create folders
from src.steps import config  # Importing configuration


def run_multiple_test(num_tests=1):
    """
    Run multiple tests of RRT-based path planning.

    Args:
        num_tests (int): Number of tests to run (default is 1).
    """
    try:
        data_loaded = load_files.load_solution_data()  # Load solution data
        ordered_transformed = shift_positions(data_loaded['ordered_positions'])  # Shift positions
        indexes = [0, 4]
        array_for_test = [ordered_transformed[i] for i in indexes]  # Select positions for testing
        name_folder = create_folder.create_folder("../../solutions")  # Create folder for solutions
        path_solutions = "../../solutions"
        rrt_instances = []
        for i in range(num_tests):
            print("Test number: ", i)
            print("Start informed RRT Unicycle star planning")
            # Create RRT instance for Unicycle RRT planning
            RRT_PLANNER = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                              data_loaded['rows'],
                              data_loaded['ordered_positions'],
                              name_folder, path_solutions)

            self_rrt = RRT_PLANNER.rrt()
            if self_rrt is not None:
                rrt_instances.append(self_rrt)

            # Create RRT instance for RRT* planning
            RRT_PLANNER_STAR = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                                   data_loaded['rows'],
                                   data_loaded['ordered_positions'],
                                   name_folder, path_solutions)
            self_rrt_star = RRT_PLANNER_STAR.rrt_star()
            if self_rrt_star is not None:
                rrt_instances.append(self_rrt_star)

            # Create RRT instance for Informed RRT planning
            RRT_PLANNER_INFORMED = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                                       data_loaded['rows'],
                                       data_loaded['ordered_positions'],
                                       name_folder, path_solutions)
            self_rrt_informed = RRT_PLANNER_INFORMED.rrt_informed()
            if self_rrt_informed is not None:
                rrt_instances.append(self_rrt_informed)

        # draw_multiple_maps(rrt_instances) if rrt_instances else None
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def draw_multiple_maps(rrt_instances):
    """
    Draw multiple maps showing RRT-based paths.

    Args:
        rrt_instances (list): List of RRT instances.
    """
    try:
        fig, ax = plt.subplots(1, figsize=(16, 10))
        fig2, ax2 = plt.subplots(1, figsize=(16, 10))
        ax.imshow(rrt_instances[0].map_array_rgb, cmap='gray', origin='lower')
        ax2.imshow(rrt_instances[0].map_array, cmap='gray', origin='lower')

        for index_rrt, self in enumerate(rrt_instances):
            if self.found and len(self.smoot_path) <= 0:
                for index, goal in enumerate(self.path):
                    cur = goal
                    if cur.parent is not None and index == 0:
                        start = self.goals[index]
                        # Draw Trees or Sample points
                        lines = []
                        while cur.col != start.col or cur.row != start.row:
                            lines.append([cur.col, cur.row, cur.parent.col, cur.parent.row])
                            cur = cur.parent
                        lines = np.array(lines)
                        color_ = "#{:02x}{:02x}{:02x}".format(np.random.randint(0, 255), np.random.randint(0, 255),
                                                              np.random.randint(0, 255))
                        colors = ['y', 'b', 'g']
                        ax.plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], color=colors[index_rrt],
                                label=f'{self.name_method}')
                        ax2.plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], color=colors[index_rrt],
                                 label=f'{self.name_method}')
                        if goal and index_rrt == 0:
                            ax.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g',
                                    label='Start')
                            ax2.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g',
                                     label='Start')
                            ax.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                    label='Goal')
                            ax2.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                     label='Goal')

            smoot_path = self.smoot_path
            if self.name_method == 'IRRT' and config.DRAW_SAFE_PATH and smoot_path is not None:
                ax.plot(smoot_path[:, 0], smoot_path[:, 1], color='r', label=self.name_method)
                ax2.plot(smoot_path[:, 0], smoot_path[:, 1], color='r', label=self.name_method)
        name_occupancy = self.name_method + '-Occupancy-Grid'
        name_rgb = self.name_method + '-RGB-Map'
        ax.legend(bbox_to_anchor=(1.01, 1), fontsize="11.5", shadow=True, borderpad=1, loc='upper left')
        ax2.legend(bbox_to_anchor=(1.01, 1), fontsize="11.5", shadow=True, borderpad=1, loc='upper left')
        ax.set_xlabel('x-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax.set_ylabel('y-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax2.set_xlabel('x-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax2.set_ylabel('y-coordinates (px)', labelpad=8, fontsize=self.font_size)
        filename_rgb = save_files.get_name_to_save_plot(self.name_folder, name_rgb, self.path_solutions, '.svg')
        filename_og = save_files.get_name_to_save_plot(self.name_folder, name_occupancy, self.path_solutions, '.svg')
        fig.savefig(filename_rgb, format='svg', dpi=500)
        fig2.savefig(filename_og, format='svg', dpi=500)
        fig.show()
        fig2.show()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


if __name__ == '__main__':
    run_multiple_test()
