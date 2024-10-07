import math
import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from scipy.interpolate import splprep, splev
from scipy.signal import savgol_filter

show_animation_2d = False
show_animation_3d = False
debug_visualization = False
plot_waypoints = False
num_waypoints = 500

class RRTStar:
    class Node:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None
            self.cost = 0.0
            self.children = []

    def __init__(self, start, goal, obstacle_list, gate_list, rand_area,
                 expand_dis=0.5, path_resolution=0.05, goal_sample_rate=10, max_iter=1000,
                 connect_circle_dist=8.335):
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.gate_list = gate_list
        self.node_list = []
        self.connect_circle_dist = connect_circle_dist
        self.animation_objects = []
        self.current_gate_index = -1
        self.next_gate_index = 0
        self.drone_size = [0.35, 0.35, 0.15]

        self.fig = None
        self.ax1 = None
        self.ax2 = None

        self.compute_sampling_area()

    def get_gate_entry_exit_points(self, gate):
        x, y, z, size, direction, entry_direction = gate
        if direction == 'X':
            if entry_direction == '+':
                entry_point = (x - 0.25, y, z)
                exit_point = (x + 0.25, y, z)
            else:
                entry_point = (x + 0.25, y, z)
                exit_point = (x - 0.25, y, z)
        else:  # 'Y'
            if entry_direction == '+':
                entry_point = (x, y - 0.25, z)
                exit_point = (x, y + 0.25, z)
            else:
                entry_point = (x, y + 0.25, z)
                exit_point = (x, y - 0.25, z)
        return entry_point, exit_point

    def compute_sampling_area(self):
        gate_x = [gate[0] for gate in self.gate_list]
        gate_y = [gate[1] for gate in self.gate_list]

        all_x = [self.start.x, self.end.x] + gate_x
        all_y = [self.start.y, self.end.y] + gate_y

        margin = 0.335
        self.sampling_area = {
            'min_x': min(all_x) - margin,
            'max_x': max(all_x) + margin,
            'min_y': min(all_y) - margin,
            'max_y': max(all_y) + margin
        }

    def planning(self, animation=True):
        self.node_list = [self.start]

        if animation and self.fig is None:
            self.setup_animation()

        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if new_node and self.check_collision(new_node, self.obstacle_list) and self.check_gate_collision(new_node):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

            if animation and i % 5 == 0:
                self.update_animation()

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y, self.node_list[-1].z) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if final_node and self.check_collision(final_node, self.obstacle_list) and self.check_gate_collision(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, theta, phi = self.calc_distance_and_angles(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        max_z_change = 0.1  # Maximum allowed z change per step

        for _ in range(n_expand):
            predict_x = not (self.start.x == self.end.x == new_node.x)
            predict_y = not (self.start.y == self.end.y == new_node.y)
            predict_z = not (self.start.z == self.end.z == new_node.z)

            new_x = new_node.x + self.path_resolution * math.sin(phi) * math.cos(theta) if predict_x else new_node.x
            new_y = new_node.y + self.path_resolution * math.sin(phi) * math.sin(theta) if predict_y else new_node.y
            
            # Calculate the desired z change
            desired_z_change = self.path_resolution * math.cos(phi) if predict_z else 0
            
            # Limit the z change to the maximum allowed
            z_change = max(min(desired_z_change, max_z_change), -max_z_change)
            new_z = new_node.z + z_change

            # Check for gate collision before adding the new point
            temp_node = self.Node(new_x, new_y, new_z)
            temp_node.parent = new_node  # Set parent for direction check
            if not self.check_gate_collision(temp_node):
                return None

            new_node.x = new_x
            new_node.y = new_y
            new_node.z = new_z
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)

            if not self.check_collision(new_node, self.obstacle_list):
                return None

        d, _, _ = self.calc_distance_and_angles(new_node, to_node)
        if d <= self.path_resolution:
            temp_node = self.Node(to_node.x, to_node.y, to_node.z)
            temp_node.parent = new_node  # Set parent for direction check
            if self.check_gate_collision(temp_node):
                new_node.path_x.append(to_node.x)
                new_node.path_y.append(to_node.y)
                new_node.path_z.append(to_node.z)
                new_node.x = to_node.x
                new_node.y = to_node.y
                new_node.z = to_node.z

        new_node.parent = from_node
        new_node.cost = from_node.cost + d

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([node.x, node.y, node.z])
        return path

    def smooth_path(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
        if len(path) < 2:
            return path

        newpath = [[p[0], p[1], p[2]] for p in path]
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(3):  # x, y, z
                    aux = newpath[i][j]
                    newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                    newpath[i][j] += weight_smooth * (newpath[i - 1][j] + newpath[i + 1][j] - 2.0 * newpath[i][j])
                    change += abs(aux - newpath[i][j])

        return newpath

    def optimize_path_for_racing(self, path):
        # Spline interpolation
        tck, u = splprep([np.array(path)[:, 0], np.array(path)[:, 1], np.array(path)[:, 2]], s=0)
        u_new = np.linspace(0, 1, num=len(path) * 5)
        smooth_path = np.column_stack(splev(u_new, tck))

        # Smoothing
        optimized_path = []
        for i in range(len(smooth_path)):
            point = smooth_path[i]
            if i > 0 and i < len(smooth_path) - 1:
                prev_point = smooth_path[i - 1]
                next_point = smooth_path[i + 1]
                direction = (next_point - prev_point) / np.linalg.norm(next_point - prev_point)
                optimized_point = point + direction * 0.067
                node = self.Node(*optimized_point)
                node.path_x = [optimized_point[0]]
                node.path_y = [optimized_point[1]]
                node.path_z = [optimized_point[2]]
                if self.check_collision(node, self.obstacle_list) and self.check_gate_collision(node):
                    optimized_path.append(optimized_point)
                else:
                    optimized_path.append(point)
            else:
                optimized_path.append(point)

        return optimized_path

    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        predict_z = not (self.start.z == self.end.z == z)
        dz = z - self.end.z if predict_z else 0
        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            predict_z = self.start.z != self.end.z
            rnd = self.Node(
                random.uniform(self.sampling_area['min_x'], self.sampling_area['max_x']),
                random.uniform(self.sampling_area['min_y'], self.sampling_area['max_y']),
                random.uniform(self.min_rand, self.max_rand) if predict_z else self.start.z
            )
        else:
            rnd = self.Node(self.end.x, self.end.y, self.end.z)
        return rnd

    def setup_animation(self):
        if self.fig is not None:
            return

        plt.ion()
        if show_animation_2d and show_animation_3d:
            self.fig = plt.figure(figsize=(12, 5))
            self.ax1 = self.fig.add_subplot(121)
            self.ax2 = self.fig.add_subplot(122, projection='3d')
        elif show_animation_2d:
            self.fig, self.ax1 = plt.subplots(figsize=(6, 5))
            self.ax2 = None
        elif show_animation_3d:
            self.fig = plt.figure(figsize=(6, 5))
            self.ax2 = self.fig.add_subplot(111, projection='3d')
            self.ax1 = None

        self.setup_plot_2d()
        self.setup_plot_3d()

    def setup_plot_2d(self):
        if not show_animation_2d:
            return
        self.ax1.clear()
        self.ax1.set_xlim(self.sampling_area['min_x'] - 0.33, self.sampling_area['max_x'] + 0.33)
        self.ax1.set_ylim(self.sampling_area['min_y'] - 0.33, self.sampling_area['max_y'] + 0.33)
        self.ax1.grid(True)
        self.ax1.set_title('2D View (XY Plane)')
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        for gate in self.gate_list:
            self.plot_gate_2d(self.ax1, gate)
        self.ax1.plot(self.start.x, self.start.y, "xr")
        self.ax1.plot(self.end.x, self.end.y, "xr")
        self.path_2d, = self.ax1.plot([], [], '-g')
        self.animation_objects.append(self.path_2d)
        self.plot_obstacles_2d(self.ax1)
        self.ax1.set_aspect('equal', 'box')

    def setup_plot_3d(self):
        if not show_animation_3d:
            return
        self.ax2.clear()
        self.ax2.set_xlim(self.sampling_area['min_x'] - 0.33, self.sampling_area['max_x'] + 0.33)
        self.ax2.set_ylim(self.sampling_area['min_y'] - 0.33, self.sampling_area['max_y'] + 0.33)
        self.ax2.set_zlim(0, 1.67)
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Z')
        self.ax2.set_title('3D View')
        for gate in self.gate_list:
            self.plot_gate_3d(self.ax2, gate)
        self.ax2.scatter(self.start.x, self.start.y, self.start.z, marker='x', color='r', s=100, label='Start')
        self.ax2.scatter(self.end.x, self.end.y, self.end.z, marker='x', color='r', s=100, label='Goal')
        self.path_3d, = self.ax2.plot([], [], [], '-g')
        self.animation_objects.append(self.path_3d)
        self.plot_obstacles_3d(self.ax2)

    def update_animation(self):
        x_list = [node.x for node in self.node_list]
        y_list = [node.y for node in self.node_list]
        z_list = [node.z for node in self.node_list]

        if show_animation_2d:
            self.path_2d.set_data(x_list, y_list)

        if show_animation_3d:
            self.path_3d.set_data(x_list, y_list)
            self.path_3d.set_3d_properties(z_list)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_gate_2d(self, ax, gate):
        x, y, z, size, direction, entry_direction = gate
        entry_point, exit_point = self.get_gate_entry_exit_points(gate)
        if direction == 'X':
            ax.plot([x, x], [y-0.165, y+0.165], 'r-', linewidth=2)
            if entry_direction == '+':
                ax.arrow(x-0.25, y, 0.165, 0, head_width=0.05, head_length=0.05, fc='r', ec='r')
            else:
                ax.arrow(x+0.25, y, -0.165, 0, head_width=0.05, head_length=0.05, fc='r', ec='r')
        else:  # 'Y'
            ax.plot([x-0.165, x+0.165], [y, y], 'r-', linewidth=2)
            if entry_direction == '+':
                ax.arrow(x, y-0.25, 0, 0.165, head_width=0.05, head_length=0.05, fc='r', ec='r')
            else:
                ax.arrow(x, y+0.25, 0, -0.165, head_width=0.05, head_length=0.05, fc='r', ec='r')
        ax.plot(entry_point[0], entry_point[1], 'g*', markersize=8)
        ax.plot(exit_point[0], exit_point[1], 'r*', markersize=8)

    def plot_gate_3d(self, ax, gate):
        x, y, z, size, direction, entry_direction = gate
        entry_point, exit_point = self.get_gate_entry_exit_points(gate)
        if direction == 'X':
            if entry_direction == '+':
                ax.quiver(x-0.25, y, z, 0.165, 0, 0, color='r', length=1, arrow_length_ratio=0.3)
            else:
                ax.quiver(x+0.25, y, z, -0.165, 0, 0, color='r', length=1, arrow_length_ratio=0.3)
        else:  # 'Y'
            if entry_direction == '+':
                ax.quiver(x, y-0.25, z, 0, 0.165, 0, color='r', length=1, arrow_length_ratio=0.3)
            else:
                ax.quiver(x, y+0.25, z, 0, -0.165, 0, color='r', length=1, arrow_length_ratio=0.3)
        theta = np.linspace(0, 2*np.pi, 100)
        if direction == 'X':
            y_circle = y + 0.165 * np.cos(theta)
            z_circle = z + 0.165 * np.sin(theta)
            ax.plot(np.full_like(theta, x), y_circle, z_circle, color='r')
            y_inner = y + 0.15 * np.cos(theta)
            z_inner = z + 0.15 * np.sin(theta)
            ax.plot(np.full_like(theta, x), y_inner, z_inner, color='r')
        else:  # 'Y'
            x_circle = x + 0.165 * np.cos(theta)
            z_circle = z + 0.165 * np.sin(theta)
            ax.plot(x_circle, np.full_like(theta, y), z_circle, color='r')
            x_inner = x + 0.15 * np.cos(theta)
            z_inner = z + 0.15 * np.sin(theta)
            ax.plot(x_inner, np.full_like(theta, y), z_inner, color='r')
        ax.scatter(entry_point[0], entry_point[1], entry_point[2], color='g', s=50, marker='*')
        ax.scatter(exit_point[0], exit_point[1], exit_point[2], color='r', s=50, marker='*')

    def plot_obstacles_2d(self, ax):
        for (ox, oy, oz, radius, height) in self.obstacle_list:
            circle = Circle((ox, oy), radius, fill=True, color='r', alpha=0.3)
            ax.add_artist(circle)

    def plot_obstacles_3d(self, ax):
        for (ox, oy, oz, radius, height) in self.obstacle_list:
            z = np.linspace(oz, oz + height, 50)
            theta = np.linspace(0, 2 * np.pi, 50)
            theta_grid, z_grid = np.meshgrid(theta, z)
            x_grid = radius * np.cos(theta_grid) + ox
            y_grid = radius * np.sin(theta_grid) + oy
            ax.plot_surface(x_grid, y_grid, z_grid, color='r', alpha=0.3)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 + (node.z - rnd_node.z) ** 2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angles(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        theta = math.atan2(dy, dx)
        phi = math.atan2(math.sqrt(dx ** 2 + dy ** 2), dz)
        return d, theta, phi

    def check_collision(self, node, obstacleList):
        if node is None or not node.path_x:
            return False
        # Check collision with obstacles
        for (ox, oy, oz, radius, height) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            dz_list = [oz - z for z in node.path_z]
            d_list = [math.sqrt(dx ** 2 + dy ** 2) for (dx, dy) in zip(dx_list, dy_list)]

            if any(d <= radius + max(self.drone_size[:2]) / 2 and oz <= z <= oz + height for d, z in zip(d_list, node.path_z)):
                return False

        # Check collision with gates
        for gate in self.gate_list:
            x, y, z, size, direction, entry_direction = gate
            ring_thickness = 0.1 # 0.05?
            inner_radius = size / 2 - ring_thickness
            outer_radius = size / 2 + ring_thickness

            for x_node, y_node, z_node in zip(node.path_x, node.path_y, node.path_z):
                if direction == 'X':
                    dx = abs(x_node - x)
                    if dx <= ring_thickness:
                        dy = y_node - y
                        dz = z_node - z
                        distance = math.sqrt(dy ** 2 + dz ** 2)
                        if inner_radius <= distance <= outer_radius:
                            return False
                else:  # 'Y'
                    dy = abs(y_node - y)
                    if dy <= ring_thickness:
                        dx = x_node - x
                        dz = z_node - z
                        distance = math.sqrt(dx ** 2 + dz ** 2)
                        if inner_radius <= distance <= outer_radius:
                            return False

        return True

        
    def check_gate_collision(self, node):
        if node.parent is None:
            return True

        for gate in self.gate_list:
            x, y, z, size, direction, entry_direction = gate

            if direction == 'X':
                if entry_direction == '+':
                    gate_normal = np.array([1, 0, 0])
                else:
                    gate_normal = np.array([-1, 0, 0])
                gate_plane_pos = x
                p0_pos = node.parent.x
                p1_pos = node.x
            else:  # 'Y'
                if entry_direction == '+':
                    gate_normal = np.array([0, 1, 0])
                else:
                    gate_normal = np.array([0, -1, 0])
                gate_plane_pos = y
                p0_pos = node.parent.y
                p1_pos = node.y

            if (p0_pos - gate_plane_pos) * (p1_pos - gate_plane_pos) <= 0:
                denom = p1_pos - p0_pos
                if denom == 0:
                    t = 0
                else:
                    t = (gate_plane_pos - p0_pos) / denom
                p0 = np.array([node.parent.x, node.parent.y, node.parent.z])
                p1 = np.array([node.x, node.y, node.z])
                intersection_point = p0 + t * (p1 - p0)

                if direction == 'X':
                    dy = intersection_point[1] - y
                    dz = intersection_point[2] - z
                    distance = math.sqrt(dy ** 2 + dz ** 2)
                    if distance <= size / 2 + 0.1:
                        movement_vector = p1 - p0
                        dot_product = np.dot(movement_vector, gate_normal)
                        if dot_product <= 0:
                            return False
                else:  # 'Y'
                    dx = intersection_point[0] - x
                    dz = intersection_point[2] - z
                    distance = math.sqrt(dx ** 2 + dz ** 2)
                    if distance <= size / 2 + 0.1:
                        movement_vector = p1 - p0
                        dot_product = np.dot(movement_vector, gate_normal)
                        if dot_product <= 0:
                            return False

        return True
        

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        r = min(r, self.expand_dis * 5)
        dist_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 + (node.z - new_node.z) ** 2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list) and self.check_gate_collision(t_node):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))
        min_cost = min(costs)

        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angles(from_node, to_node)
        return from_node.cost + d

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list) and self.check_gate_collision(edge_node)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                if near_node.parent:
                    if near_node in near_node.parent.children:
                        near_node.parent.children.remove(near_node)
                near_node.parent = new_node
                near_node.cost = edge_node.cost
                new_node.children.append(near_node)
                self.propagate_cost_to_leaves(near_node)

    def propagate_cost_to_leaves(self, parent_node):
        for child in parent_node.children:
            child.cost = self.calc_new_cost(parent_node, child)
            self.propagate_cost_to_leaves(child)

def calculate_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)

def print_scenario_details(scenario, gate_list, gate_order, start, end, obstacle_list, debug_visualization=False):
    print(f"\nScenario: {scenario}")
    print("Gate List:")
    for i, gate in enumerate(gate_list):
        print(f"  Gate {i+1}: {gate}")
    print("Gate Order:", [str(g) if isinstance(g, int) else g for g in gate_order])
    print(f"Start: {start}")
    print(f"End: {end}")
    print("Obstacles:")
    for obstacle in obstacle_list:
        print(f"  {obstacle}")

    if debug_visualization:
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(-0.5, 3.5)
        ax.set_ylim(-0.5, 3.5)
        ax.set_aspect('equal')
        ax.grid(True)

        # Plot gates
        for i, gate in enumerate(gate_list):
            x, y, z, size, direction, entry = gate
            if direction == 'X':
                ax.plot([x, x], [y-size/2, y+size/2], 'r-', linewidth=2)
            else:  # 'Y'
                ax.plot([x-size/2, x+size/2], [y, y], 'r-', linewidth=2)
            ax.text(x, y, f'{i+1}', fontsize=12, ha='center', va='center')

        # Plot start and end points
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(end[0], end[1], 'bo', markersize=10, label='End')

        # Plot obstacles
        for obstacle in obstacle_list:
            ox, oy, oz, radius, height = obstacle
            circle = Circle((ox, oy), radius, fill=True, color='gray', alpha=0.5)
            ax.add_artist(circle)
            ax.text(ox, oy, f'H:{height}', fontsize=10, ha='center', va='center')

        # Plot gate order
        for i in range(len(gate_order) - 1):
            if gate_order[i] == 'START':
                x1, y1 = start[:2]
            elif gate_order[i] == 'END':
                x1, y1 = end[:2]
            else:
                x1, y1 = gate_list[gate_order[i]][:2]

            if gate_order[i+1] == 'START':
                x2, y2 = start[:2]
            elif gate_order[i+1] == 'END':
                x2, y2 = end[:2]
            else:
                x2, y2 = gate_list[gate_order[i+1]][:2]

            ax.arrow(x1, y1, x2-x1, y2-y1, head_width=0.05, head_length=0.1, fc='g', ec='g', alpha=0.5)

        ax.legend()
        plt.title(f"{scenario} Scenario")
        plt.show()

def create_gate_list(scenario):
    if scenario == 'circle':
        return [
            (1.5, 0, 1, 0.395, 'X', '+'),  # Gate 1
            (3, 1.5, 0.5, 0.395, 'Y', '+'),  # Gate 2
            (1.5, 3, 1, 0.395, 'X', '-'),  # Gate 3
            (0, 1.5, 0.5, 0.395, 'Y', '-'),  # Gate 4
        ]
    elif scenario == 'circle-constant-z':
        return [
            (1.5, 0, 0.8, 0.395, 'X', '+'),  # Gate 1
            (3, 1.5, 0.8, 0.395, 'Y', '+'),  # Gate 2
            (1.5, 3, 0.8, 0.395, 'X', '-'),  # Gate 3
            (0, 1.5, 0.8, 0.395, 'Y', '-'),  # Gate 4
        ]
    elif scenario == 'figure-eight':
        return [
            (0.75, 0, 1, 0.395, 'X', '+'),  # Gate 1
            (1.5, 0.75, 1, 0.395, 'Y', '+'),  # Gate 2
            (0.75, 1.5, 1, 0.395, 'X', '-'),  # Gate 3
            (0, 0.75, 1, 0.395, 'Y', '-'),  # Gate 4
            (2.25, 1.5, 1, 0.395, 'X', '+'),  # Gate 5
            (3, 2.25, 1, 0.395, 'Y', '+'),  # Gate 6
            (2.25, 3, 1, 0.395, 'X', '-'),  # Gate 7
            (1.5, 2.25, 1, 0.395, 'Y', '-'),  # Gate 8
        ]
    elif scenario == 'u-shape':
        return [
            (0, 1.5, 1, 0.395, 'Y', '+'),  # Gate 1
            (1.5, 0, 1, 0.395, 'X', '+'),  # Gate 2
            (3, 1.5, 1, 0.395, 'Y', '-'),  # Gate 3
        ]
    elif scenario == 'u-shape-2':
        return [
            (1.5, 0, 1, 0.395, 'X', '+'),  # Gate 1
        ]
    elif scenario == 's-shape':
        return [
            (0.5, 0.5, 1, 0.395, 'Y', '+'),  # Gate 1
            (1.5, 1, 1, 0.395, 'Y', '-'),  # Gate 2
            (0.5, 1.5, 1, 0.395, 'Y', '+'),  # Gate 3
            (1.5, 2, 1, 0.395, 'Y', '-'),  # Gate 4
        ]
    else:
        raise ValueError("Invalid scenario")

def create_gate_order(scenario):
    if scenario == 'circle' or scenario == 'circle-constant-z':
        return ['START', 0, 1, 2, 3, 0, 'END']
    elif scenario == 'figure-eight':
        return ['START', 0, 1, 2, 3, 0, 1, 4, 5, 6, 7, 4, 'END']
    elif scenario == 'u-shape':
        return ['START', 0, 1, 2, 'END']
    elif scenario == 'u-shape-2':
        return ['START', 0, 'END']
    elif scenario == 's-shape':
        return ['START', 0, 1, 2, 3, 'END']
    else:
        raise ValueError("Invalid scenario")

def create_obstacle_list(scenario):
    if scenario in ['circle', 'circle-constant-z', 'u-shape', 'u-shape-2']:
        return [(1.5, 1.5, 0, 1.3, 3.0)]  # (x, y, z, radius, height)
    elif scenario == 'figure-eight':
        return [
            (0.75, 0.75, 0, 0.55, 3.0),  # Obstacle in the first circle
            (2.25, 2.25, 0, 0.55, 3.0)   # Obstacle in the second circle
        ]
    elif scenario == 's-shape':
        return []
    else:
        raise ValueError("Invalid scenario")

def main_func(num_waypoints=500, scenario='circle', debug_visualization=False):
    print(f"Start path planning for {scenario} scenario")

    gate_list = create_gate_list(scenario)
    gate_order = create_gate_order(scenario)
    obstacle_list = create_obstacle_list(scenario)

    start = [0, 0, 0.8]  # Start point
    end = [3, 0, 0.8]  # End point
    if scenario == 'u-shape-2':
        start = [0, 1.5, 1]
        end = [3, 1.5, 1]

    print_scenario_details(scenario, gate_list, gate_order, start, end, obstacle_list, debug_visualization)

    rrt_star = RRTStar(
        start=start,
        goal=end,
        rand_area=[-0.5, 3.5],
        obstacle_list=obstacle_list,
        gate_list=gate_list,
        expand_dis=0.2,
        path_resolution=0.05,
        goal_sample_rate=10,
        max_iter=5000
    )

    # Create waypoints based on gate_order
    waypoints = []
    for point in gate_order:
        if point == 'START':
            waypoints.append(start)
        elif point == 'END':
            waypoints.append(end)
        else:
            gate = gate_list[point]
            entry_point, exit_point = rrt_star.get_gate_entry_exit_points(gate)
            waypoints.append(entry_point)
            waypoints.append(gate[:3])  # Gate center
            waypoints.append(exit_point)

    full_path = []

    for i in range(len(waypoints) - 1):
        start_point = waypoints[i]
        end_point = waypoints[i + 1]

        rrt_star.start = rrt_star.Node(*start_point)
        rrt_star.end = rrt_star.Node(*end_point)

        rrt_star.current_gate_index = i
        rrt_star.next_gate_index = (i + 1) % (len(waypoints) - 1)  # Exclude END from cycling

        rrt_star.compute_sampling_area()

        max_attempts = 5
        path = None
        for attempt in range(max_attempts):
            path = rrt_star.planning(animation=False)
            if path is not None:
                break
            print(f"Attempt {attempt + 1} failed, retrying...")

        if path is None:
            print(f"Cannot find path from {start_point} to {end_point} after {max_attempts} attempts")
            return

        if i > 0:
            path = path[1:]
        full_path.extend(path[::-1])

    print("Found initial path!")

    smoothed_path = rrt_star.smooth_path(full_path)
    print("Path smoothed")
    racing_path = rrt_star.optimize_path_for_racing(smoothed_path)
    print("Path optimized for racing")
    final_optimized_path = racing_path

    final_optimized_path[-1] = end

    gate_indices = []
    for point in gate_order:
        if point == 'START':
            gate_indices.append(0)
        elif point == 'END':
            gate_indices.append(len(final_optimized_path) - 1)
        else:
            gate = gate_list[point]
            distances = [np.linalg.norm(np.array(gate[:3]) - np.array(p)) for p in final_optimized_path]
            closest_index = np.argmin(distances)
            gate_indices.append(closest_index)
    gate_indices = sorted(list(set(gate_indices)))

    num_generated_waypoints = num_waypoints - len(gate_indices)
    
    interpolated_path = []
    for i in range(len(gate_indices) - 1):
        start_idx = gate_indices[i]
        end_idx = gate_indices[i + 1]
        segment = final_optimized_path[start_idx:end_idx + 1]
        num_points = int(num_generated_waypoints * (end_idx - start_idx) / (len(final_optimized_path) - 1)) + 1
        
        t = np.linspace(0, 1, num_points)
        x = np.interp(t, np.linspace(0, 1, len(segment)), [p[0] for p in segment])
        y = np.interp(t, np.linspace(0, 1, len(segment)), [p[1] for p in segment])
        z = np.interp(t, np.linspace(0, 1, len(segment)), [p[2] for p in segment])
        
        interpolated_segment = list(zip(x, y, z))
        interpolated_path.extend(interpolated_segment)

    final_path = interpolated_path

    final_path[-1] = end

    angles = []
    for i in range(len(final_path) - 1):
        angle = calculate_angle(final_path[i], final_path[i+1])
        angles.append(angle)
    
    angles.append(angles[-1])
    
    smoothed_angles = np.arctan2(np.sin(angles), np.cos(angles))

    waypoints_with_theta = []
    for i in range(len(final_path)):
        waypoints_with_theta.append([final_path[i][0], final_path[i][1], final_path[i][2], smoothed_angles[i]])

    if show_animation_2d or show_animation_3d:
        x_list = [x for (x, y, z, _) in waypoints_with_theta]
        y_list = [y for (x, y, z, _) in waypoints_with_theta]
        z_list = [z for (x, y, z, _) in waypoints_with_theta]

        if rrt_star.fig is None:
            rrt_star.setup_animation()

        if show_animation_2d and rrt_star.ax1 is not None:
            rrt_star.ax1.cla()
            rrt_star.setup_plot_2d()
            rrt_star.ax1.plot(x_list, y_list, '-b')
            rrt_star.ax1.plot(x_list[0], y_list[0], 'go', markersize=10, label='Start')
            rrt_star.ax1.plot(x_list[-1], y_list[-1], 'bo', markersize=10, label='End')
            
            arrow_indices = np.linspace(0, len(x_list) - 1, 20, dtype=int)
            for i in arrow_indices:
                x, y, z, theta = waypoints_with_theta[i]
                dx = math.cos(theta) * 0.2
                dy = math.sin(theta) * 0.2
                rrt_star.ax1.arrow(x, y, dx, dy, 
                                head_width=0.05, head_length=0.1, 
                                fc='r', ec='r', alpha=0.5)
            
            if plot_waypoints:
                rrt_star.ax1.plot(x_list, y_list, 'ro', markersize=2, label='Waypoints')
            
            rrt_star.ax1.legend()

        if show_animation_3d and rrt_star.ax2 is not None:
            rrt_star.ax2.cla()
            rrt_star.setup_plot_3d()
            rrt_star.ax2.plot(x_list, y_list, z_list, '-b')
            rrt_star.ax2.scatter(x_list[0], y_list[0], z_list[0], color='g', s=100, label='Start')
            rrt_star.ax2.scatter(x_list[-1], y_list[-1], z_list[-1], color='b', s=100, label='End')
            
            arrow_indices = np.linspace(0, len(x_list) - 1, 20, dtype=int)
            for i in arrow_indices:
                if i + 1 < len(x_list):
                    rrt_star.ax2.quiver(x_list[i], y_list[i], z_list[i],
                                        x_list[i+1] - x_list[i],
                                        y_list[i+1] - y_list[i],
                                        z_list[i+1] - z_list[i],
                                        color='r', alpha=0.5, arrow_length_ratio=0.15)
            
            if plot_waypoints:
                rrt_star.ax2.scatter(x_list, y_list, z_list, color='r', s=10, label='Waypoints')
            
            rrt_star.ax2.legend()

        plt.tight_layout()
        plt.ioff()
        plt.show()

    path_length = sum(np.linalg.norm(np.array(waypoints_with_theta[i][:3]) - np.array(waypoints_with_theta[i - 1][:3]))
                      for i in range(1, len(waypoints_with_theta)))
    print(f"Final path length: {path_length:.2f}")
    print(f"Number of waypoints: {len(waypoints_with_theta)}")
    
    return waypoints_with_theta

if __name__ == '__main__':
    scenario = 'u-shape-2'
    result = main_func(num_waypoints=num_waypoints, scenario=scenario, debug_visualization=debug_visualization)
    if True or (not show_animation_2d and not show_animation_3d):
        print("Waypoints:")
        for waypoint in result:
            print(f"[{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}, {waypoint[3]:.2f}]")

# if __name__ == '__main__':
#     scenarios = ['circle', 'circle-constant-z', 'figure-eight', 'u-shape', 's-shape']
#     for scenario in scenarios:
#         result = main_func(num_waypoints=num_waypoints, scenario=scenario, debug_visualization=debug_visualization)
#         print(f"Completed {scenario} scenario")
#         if not show_animation_2d and not show_animation_3d:
#             print("Waypoints:")
#             for waypoint in result:
#                 print(f"[{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}, {waypoint[3]:.2f}]")
#         print("\n")