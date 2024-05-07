import numpy as np
import traceback
from colorama import init, Fore
import matplotlib.pyplot as plt
from src.components.common import load_files, save_files
import matplotlib.lines as mlines
from shapely.geometry import LineString

from src.components.step_3_rrt.smooth_path import smooth_path
from src.steps import config


def draw_map(self):
    """
    Draw the occupancy grid and RGB map along with the planned path and relevant statistics.

    Args:
        self: Reference to the class instance.
    """
    try:
        fig, ax = plt.subplots(1, figsize=(12, 6))
        fig2, ax2 = plt.subplots(1, figsize=(12, 6))
        ax.imshow(self.map_array_rgb, cmap='gray', origin='lower')
        ax2.imshow(self.map_array, cmap='gray', origin='lower')

        if self.found and len(self.smoot_path) <= 0:
            if config.DRAW_TREE_RRT and config.BREAK_AT == 1:
                for node in self.vertices[1:-1]:
                    ax.plot(node.col, node.row, markersize=3, marker='o', color='y')
                    ax.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
                    ax2.plot(node.col, node.row, markersize=3, marker='o', color='y')
                    ax2.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')

            for index, (goal) in enumerate(self.path):
                cur = goal
                if cur.parent is not None:
                    start = self.goals[index]
                    lines = []
                    while cur.col != start.col or cur.row != start.row:
                        lines.append((cur.col, cur.row))
                        cur = cur.parent
                    lines.append((start.col, start.row))
                    lines.reverse()
                    lines = np.array(lines)
                    color_ = "#{:02x}{:02x}{:02x}".format(np.random.randint(0, 255),
                                                          np.random.randint(0, 255),
                                                          np.random.randint(0, 255))
                    ax.plot(lines[:, 0], lines[:, 1], color='b')
                    ax2.plot(lines[:, 0], lines[:, 1], color='b')
                    if goal:
                        ax.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                label='Goal')
                        ax2.plot(goal.col, goal.row, markersize=5, marker='o', color='r',
                                 label='Goal')

        # Draw Trees or Sample points
        if config.DRAW_TREE_RRT and config.BREAK_AT == 1 and self.name_method == 'IRRT':
            for node in self.vertices[1:-1]:
                ax.plot(node.col, node.row, markersize=3, marker='o', color='y')
                ax.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
                ax2.plot(node.col, node.row, markersize=3, marker='o', color='y')
                ax2.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        smoot_path = self.smoot_path
        if self.name_method == 'IRRT' and smoot_path is not None:
            ax.plot(smoot_path[:, 0], smoot_path[:, 1], color='b', label=self.name_method)
            ax2.plot(smoot_path[:, 0], smoot_path[:, 1], color='b', label=self.name_method)
            if config.DRAW_SAFE_PATH:
                line = LineString(self.smoot_path)
                parallel_line = line.parallel_offset(config.CLEARANCE_RADIUS, side='right')
                parallel_line_left = line.parallel_offset(config.CLEARANCE_RADIUS,
                                                          side='left')
                parallel_x, parallel_y = parallel_line.xy
                parallel_x_left, parallel_y_left = parallel_line_left.xy
                ax.plot(parallel_x, parallel_y, color='black', linestyle='--', linewidth=1.5)
                ax.plot(parallel_x_left, parallel_y_left, color='black', linestyle='--',
                        linewidth=1.5)
                ax2.plot(parallel_x, parallel_y, color='black', linestyle='--', linewidth=1.5)
                ax2.plot(parallel_x_left, parallel_y_left, color='black', linestyle='--',
                         linewidth=1.5)
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
        name_occupancy = self.name_method + '-Occupancy-Grid'
        name_rgb = self.name_method + '-RGB-Map'
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
        filename_rgb = save_files.get_name_to_save_plot(self.name_folder, name_rgb, self.path_solutions, '.svg')
        filename_og = save_files.get_name_to_save_plot(self.name_folder, name_occupancy, self.path_solutions, '.svg')
        fig.savefig(filename_rgb, format='svg', dpi=500)
        fig2.savefig(filename_og, format='svg', dpi=500)
        fig.show()
        fig2.show()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def draw_combined_maps(self):
    """
    Draw combined occupancy grid and RGB map along with the planned path and relevant statistics.

    Args:
        self: Reference to the class instance.
    """
    try:
        fig, axs = plt.subplots(1, 2, figsize=(12, 6))  # Dos subplots en una fila

        # Subplot 1: Mapa de cuadrícula de ocupación
        axs[0].imshow(self.map_array, cmap='gray', origin='lower')
        axs[1].imshow(self.map_array_rgb, cmap='gray', origin='lower')

        if self.found:
            for index, (goal) in enumerate(self.path):
                cur = goal
                if cur.parent is not None:
                    start = self.goals[index]
                    lines = []
                    while cur.col != start.col or cur.row != start.row:
                        lines.append([cur.col, cur.row, cur.parent.col, cur.parent.row])
                        cur = cur.parent
                    lines = np.array(lines)
                    color_ = "#{:02x}{:02x}{:02x}".format(np.random.randint(0, 255),
                                                          np.random.randint(0, 255),
                                                          np.random.randint(0, 255))
                    axs[0].plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], color=color_)
                    axs[1].plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], color=color_)
                    if goal:
                        axs[0].plot(goal.col, goal.row, markersize=5, marker='o', color=color_,
                                    label='Goal')
                        axs[1].plot(goal.col, goal.row, markersize=5, marker='o', color=color_,
                                    label='Goal')

            axs[0].plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g', label='Start')
            axs[1].plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g', label='Start')

            custom_texts = [
                mlines.Line2D([], [], color='black', marker='o', linestyle='',
                              label=f"Total nodes: " + str(self.total_nodes)),
                mlines.Line2D([], [], color='black', marker='o', linestyle='',
                              label=f"Cost: " + "{:.2f}".format(self.total_cost) + "(px)"),
                mlines.Line2D([], [], color='black', marker='o', linestyle='',
                              label=f"Collisions: " + "{:.0f}".format(
                                  self.total_collisions) + "(n)"),
                mlines.Line2D([], [], color='black', marker='o', linestyle='',
                              label=f"Samples: " + "{:.2f}".format(self.total_samples)),
                mlines.Line2D([], [], color='black', marker='o', linestyle='',
                              label=f"Planning time: " + "{:.2f}".format(self.total_planning_time) + "(s)")

            ]
            axs[0].set_title(self.name_method + '-Occupancy-Grid', fontsize=18)
            axs[0].set_xlabel('x-coordinates (px)', labelpad=8, fontsize=18)
            axs[0].set_ylabel('y-coordinates (px)', labelpad=8, fontsize=18)

            axs[1].set_title(self.name_method + '-RGB-Map', fontsize=18)
            axs[1].set_xlabel('x-coordinates (px)', labelpad=8, fontsize=18)
            axs[1].set_ylabel('y-coordinates (px)', labelpad=8, fontsize=18)

        plt.tight_layout()

        # Crear los dos legends
        legend_colors = axs[0].legend(loc='upper center', bbox_to_anchor=(1.1, 1), fontsize="18", shadow=True,
                                      borderpad=1)
        legend_texts = axs[0].legend(handles=custom_texts, fontsize="14", shadow=True, borderpad=1,
                                     loc='lower right', bbox_to_anchor=(2.17, 0))

        # fig.add_artist(legend_colors)
        # fig.add_artist(legend_texts)
        # plt.legend(handles=custom_texts, bbox_to_anchor=(1.01, 1), fontsize="18", shadow=True, borderpad=1,
        #            loc='upper left')
        filename = save_files.get_name_to_save_plot(self.name_folder, self.name_method, self.path_solutions)
        plt.savefig(filename, dpi=500)
        plt.show()

    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()
