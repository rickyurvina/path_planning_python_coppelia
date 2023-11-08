import matplotlib.pyplot as plt
import numpy as np
import pure_pursuit
import unicycle_model
from rrt_star_reeds_shepp import RRTStarReedsShepp
import math
from src.components.common import load_files

show_animation = True

Lf = 0.5  # look-ahead distance


class ClosedLoopRRTStar(RRTStarReedsShepp):
    """
    Class for Closed loop RRT star planning
    """

    def __init__(self, start, goal, occupancy_grid, rand_area,
                 max_iter=500,
                 connect_circle_dist=600.0,
                 robot_radius=0.4
                 ):
        super().__init__(start, goal, occupancy_grid, rand_area,
                         max_iter=max_iter,
                         connect_circle_dist=connect_circle_dist,
                         robot_radius=robot_radius
                         )

        self.target_speed = 10.0 / 3.6
        self.yaw_th = np.deg2rad(0.3)
        self.xy_th = 0.5
        self.invalid_travel_ratio = 5.0

    def planning(self, animation=True):
        """
        do planning

        animation: flag for animation on or off
        """
        # planning with RRTStarReedsShepp
        super().planning(animation=animation)

        # generate coruse
        path_indexs = self.get_goal_indexes()

        flag, x, y, yaw, v, t, a, d = self.search_best_feasible_path(
            path_indexs)

        return flag, x, y, yaw, v, t, a, d

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path

    def search_best_feasible_path(self, path_indexs):

        print("Start search feasible path")

        best_time = float("inf")

        fx, fy, fyaw, fv, ft, fa, fd = None, None, None, None, None, None, None

        # pure pursuit tracking
        for ind in path_indexs:
            path = self.generate_final_course(ind)

            flag, x, y, yaw, v, t, a, d = self.check_tracking_path_is_feasible(
                path)

            if flag and best_time >= t[-1]:
                print("feasible path is found")
                best_time = t[-1]
                fx, fy, fyaw, fv, ft, fa, fd = x, y, yaw, v, t, a, d

        print("best time is")
        print(best_time)

        if fx:
            fx.append(self.end.x)
            fy.append(self.end.y)
            fyaw.append(self.end.yaw)
            return True, fx, fy, fyaw, fv, ft, fa, fd

        return False, None, None, None, None, None, None, None

    def extend_path(self, cx, cy, cyaw):

        dl = 0.1
        dl_list = [dl] * (int(Lf / dl) + 1)

        move_direction = math.atan2(cy[-1] - cy[-3], cx[-1] - cx[-3])
        is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0

        for idl in dl_list:
            if is_back:
                idl *= -1
            cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
            cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
            cyaw = np.append(cyaw, cyaw[-1])

        return cx, cy, cyaw

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def check_collision(node, occupancy_grid, robot_radius):
        if node is None:
            return False

        for x, y in zip(node.path_x, node.path_y):
            x_idx = int(x)
            y_idx = int(y)

            if (
                    x_idx < 0
                    or x_idx >= len(occupancy_grid[0])
                    or y_idx < 0
                    or y_idx >= len(occupancy_grid)
                    or occupancy_grid[y_idx][x_idx] == 0
            ):
                print("collision")
                return False  # Collision

        return True

    def check_tracking_path_is_feasible(self, path):
        cx = np.array([state[0] for state in path])[::-1]
        cy = np.array([state[1] for state in path])[::-1]
        cyaw = np.array([state[2] for state in path])[::-1]

        goal = [cx[-1], cy[-1], cyaw[-1]]

        cx, cy, cyaw = self.extend_path(cx, cy, cyaw)

        speed_profile = pure_pursuit.calc_speed_profile(
            cx, cy, cyaw, self.target_speed)

        t, x, y, yaw, v, a, d, find_goal = pure_pursuit.closed_loop_prediction(
            cx, cy, cyaw, speed_profile, goal)
        yaw = [self.pi_2_pi(iyaw) for iyaw in yaw]

        if not find_goal:
            print("cannot reach goal")

        if abs(yaw[-1] - goal[2]) >= self.yaw_th * 10.0:
            print("final angle is bad")
            find_goal = False

        travel = unicycle_model.dt * sum(np.abs(v))
        origin_travel = sum(np.hypot(np.diff(cx), np.diff(cy)))

        if (travel / origin_travel) >= self.invalid_travel_ratio:
            print("path is too long")
            find_goal = False

        tmp_node = self.Node(x, y, 0)
        tmp_node.path_x = x
        tmp_node.path_y = y
        if not self.check_collision(
                tmp_node, self.obstacle_list, self.robot_radius):
            print("This path is collision")
            find_goal = False

        return find_goal, x, y, yaw, v, t, a, d

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_goal_indexes(self):

        goalinds = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.xy_th:
                goalinds.append(i)
        print("OK XY TH num is")
        print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.yaw_th:
                fgoalinds.append(i)
        print("OK YAW TH num is")
        print(len(fgoalinds))

        return fgoalinds

    def draw_graph_unicycle(self, rnd=None):

        fig, ax = plt.subplots(1)
        ax.imshow(self.obstacle_list, cmap='gray', origin='lower')
        # for stopping simulation with the esc key.

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        # for node in self.node_list:
        #     if node.parent:
        #         plt.plot(node.path_x, node.path_y, "-g")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def draw_graph_rrt(self, rnd=None):

        fig, ax = plt.subplots(1)
        ax.imshow(self.obstacle_list, cmap='gray', origin='lower')
        # for stopping simulation with the esc key.

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        # for node in self.node_list:
        #     if node.parent:
        #         plt.plot(node.path_x, node.path_y, "-g")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)


def main():
    print("Start" + __file__)
    # ====Search Path with RRT====
    data = load_files.load_solution_data()
    occupancy_grid = data['occupancy_grid']

    # Set Initial parameters
    start = [200.0, 200.0, np.deg2rad(0.0)]
    goal = [200, 400, np.deg2rad(-90.0)]

    closed_loop_rrt_star = ClosedLoopRRTStar(start, goal,
                                             occupancy_grid,
                                             [190.0, 410.0],
                                             max_iter=400,
                                             connect_circle_dist=100.0)
    # closed_loop_rrt_star.draw_graph_rrt()

    flag, x, y, yaw, v, t, a, d = closed_loop_rrt_star.planning(
        animation=show_animation)

    if not flag:
        print("cannot find feasible path")

    # Draw final path

    if show_animation and flag:
        closed_loop_rrt_star.draw_graph_unicycle()
        plt.plot(x, y, '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()
        plt.title("RRT")


if __name__ == '__main__':
    main()
