import uuid
from datetime import datetime
import traceback

import numpy as np
from colorama import init, Fore
from matplotlib import pyplot as plt
from src.components.common.save_on_database import save_data_rrt_test
from src.components.step_3_rrt.informed_rrt import RRT
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files, save_files
from src.components import create_folder
from src.reports.reports import average_test
from src.steps import config
import matplotlib.lines as mlines


def run_multiple_test(num_tests=1):
    try:
        unique_code = str(uuid.uuid4())
        test_number = datetime.now().strftime("%Y%m%d") + "-" + str(unique_code)
        data_loaded = load_files.load_solution_data()
        ordered_transformed = shift_positions(data_loaded['ordered_positions'])
        indexes = [0, 3]
        array_for_test = [ordered_transformed[i] for i in indexes]
        name_folder = create_folder.create_folder("../../solutions")
        path_solutions = "../../solutions"
        for i in range(num_tests):
            print("Test number: ", i)
            print("Start informed RRT Unicycle star planning")
            RRT_PLANNER = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                              data_loaded['rows'],
                              data_loaded['ordered_positions'],
                              name_folder, path_solutions)

            method, self_rrt, _ = RRT_PLANNER.rrt()
            data = {
                'prefix': 'tests_rrt',
                'method': method if method is not None else "tests_rrt",
                'test_number': test_number,
                'total_cost': self_rrt.total_cost if self_rrt.total_cost is not None else 0,
                'total_collisions': self_rrt.total_collisions if self_rrt.total_collisions is not None else 0,
                'total_samples': self_rrt.total_samples if self_rrt.total_samples is not None else 0,
                'total_planning_time': self_rrt.total_planning_time if self_rrt.total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": self_rrt.success,
                "total_nodes": self_rrt.total_nodes if self_rrt.total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt!!")

            method, self_rrt_star, _ = RRT_PLANNER.rrt_star()
            data = {
                'prefix': 'tests_rrt_star',
                'method': method if method is not None else "rrt_star",
                'test_number': test_number,
                'total_cost': self_rrt_star.total_cost if self_rrt_star.total_cost is not None else 0,
                'total_collisions': self_rrt_star.total_collisions if self_rrt_star.total_collisions is not None else 0,
                'total_samples': self_rrt_star.total_samples if self_rrt_star.total_samples is not None else 0,
                'total_planning_time': self_rrt_star.total_planning_time if self_rrt_star.total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": self_rrt_star.success,
                "total_nodes": self_rrt_star.total_nodes if self_rrt_star.total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt-star!!")

            method, self_rrt_informed, _ = RRT_PLANNER.rrt_informed()
            data = {
                'prefix': 'tests_rrt_star_informed',
                'method': method if method is not None else "tests_rrt_star_informed",
                'test_number': test_number,
                'total_cost': self_rrt_informed.total_cost if self_rrt_informed.total_cost is not None else 0,
                'total_collisions': self_rrt_informed.total_collisions if self_rrt_informed.total_collisions is not None else 0,
                'total_samples': self_rrt_informed.total_samples if self_rrt_informed.total_samples is not None else 0,
                'total_planning_time': self_rrt_informed.total_planning_time if self_rrt_informed.total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": self_rrt_informed.success,
                "total_nodes": self_rrt_informed.total_nodes if self_rrt_informed.total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
        print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt-informed!!")
        average_test(test_number, name_folder, path_solutions, num_tests)
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def draw_map(self, path, name='RRT'):
    try:
        fig, ax = plt.subplots(1, figsize=(12, 6))
        fig2, ax2 = plt.subplots(1, figsize=(12, 6))
        ax.imshow(self.map_array_rgb, cmap='gray', origin='lower')
        ax2.imshow(self.map_array, cmap='gray', origin='lower')

        if self.found:
            for index, (goal) in enumerate(path):
                cur = goal
                if cur.parent is not None:
                    start = self.goals[index]
                    # Draw Trees or Sample points
                    if config.DRAW_TREE_RRT and config.BREAK_AT == 1:
                        for node in self.vertices[1:-1]:
                            ax.plot(node.col, node.row, markersize=3, marker='o', color='y')
                            ax.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
                            ax2.plot(node.col, node.row, markersize=3, marker='o', color='y')
                            ax2.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
                    lines = []
                    while cur.col != start.col or cur.row != start.row:
                        lines.append([cur.col, cur.row, cur.parent.col, cur.parent.row])
                        cur = cur.parent
                    lines = np.array(lines)
                    color_ = "#{:02x}{:02x}{:02x}".format(np.random.randint(0, 255), np.random.randint(0, 255),
                                                          np.random.randint(0, 255))
                    ax.plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], marker='o', color='b')
                    ax2.plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], marker='o', color='b')
                    if goal:
                        ax.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                label='Goal')
                        ax2.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                 label='Goal')

        ax.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g', label='Start')
        ax2.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g', label='Start')

        custom_texts = [
            mlines.Line2D([], [], color='black', marker='o', linestyle='',
                          label=f"Total nodes: " + str(self.total_nodes)),
            mlines.Line2D([], [], color='black', marker='o', linestyle='',
                          label=f"Cost: " + "{:.2f}".format(self.total_cost) + "(px)"),
            mlines.Line2D([], [], color='black', marker='o', linestyle='',
                          label=f"Samples in obstacles: " + "{:.0f}".format(
                              self.total_collisions) + "(n)"),
            mlines.Line2D([], [], color='black', marker='o', linestyle='',
                          label=f"Samples: " + "{:.2f}".format(self.total_samples)),
            mlines.Line2D([], [], color='black', marker='o', linestyle='',
                          label=f"Planning time: " + "{:.2f}".format(self.total_planning_time) + "(s)")
        ]
        name_occupancy = name + '-Occupancy-Grid'
        name_rgb = name + '-RGB-Map'
        ax.set_title(name_rgb, fontsize=18)
        ax2.set_title(name_occupancy, fontsize=18)
        fig.legend(handles=custom_texts, fontsize="11.5", shadow=True, borderpad=1, loc='outside lower right',
                   bbox_to_anchor=(0.98, 0.3))
        fig2.legend(handles=custom_texts, fontsize="11.5", shadow=True, borderpad=1, loc='outside lower right',
                    bbox_to_anchor=(0.98, 0.3))
        ax.legend(bbox_to_anchor=(1.01, 1), fontsize="11.5", shadow=True, borderpad=1, loc='upper left')
        ax2.legend(bbox_to_anchor=(1.01, 1), fontsize="11.5", shadow=True, borderpad=1, loc='upper left')
        ax.set_xlabel('x-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax.set_ylabel('y-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax2.set_xlabel('x-coordinates (px)', labelpad=8, fontsize=self.font_size)
        ax2.set_ylabel('y-coordinates (px)', labelpad=8, fontsize=self.font_size)
        filename_rgb = save_files.get_name_to_save_plot(self.name_folder, name_rgb, self.path_solutions)
        filename_og = save_files.get_name_to_save_plot(self.name_folder, name_occupancy, self.path_solutions)
        fig.savefig(filename_rgb, dpi=500)
        fig2.savefig(filename_og, dpi=500)
        fig.show()
        fig2.show()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


if __name__ == '__main__':
    run_multiple_test()
