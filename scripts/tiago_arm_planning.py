"""
This script is used for find path to control the tool_link to the target position.

The RRT6DoFQuaternion class is used to find a path from the start to the goal position and orientation using RRT.
The calculate_map_bounds function is used to calculate the map bounds based on the start and goal positions.


Author: Yichen Xie
"""


import numpy as np
import random
from scipy.spatial.transform import Rotation as R, Slerp
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class RRT6DoFQuaternion:
    def __init__(self, start, goal, obstacle_list, map_bounds, step_size=0.025, max_iter=3000):
        self.start = {'pos': np.array(start[:3]), 'quat': np.array(start[3:]), 'parent': None}
        self.goal = {'pos': np.array(goal[:3]), 'quat': np.array(goal[3:]), 'parent': None}
        self.obstacle_list = obstacle_list
        self.map_bounds = map_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]

    

    def distance(self, node1, node2):
        pos_dist = np.linalg.norm(node1['pos'] - node2['pos'])  # Distance in x, y, z
        quat1 = R.from_quat(node1['quat'])
        quat2 = R.from_quat(node2['quat'])
        rot_dist = quat1.inv() * quat2
        angle_diff = np.linalg.norm(rot_dist.as_rotvec())
        return pos_dist + angle_diff

    def get_random_point(self, goal_bias=0.75):
        # Sample a random position within the map bounds
        random_position = np.array([
            random.uniform(self.map_bounds[0][0], self.map_bounds[0][1]),
            random.uniform(self.map_bounds[1][0], self.map_bounds[1][1]),
            random.uniform(self.map_bounds[2][0], self.map_bounds[2][1])
        ])
        
        # Bias the orientation sampling towards the goal orientation
          # With some probability, sample near the goal's orientation
        random_orientation = self.goal['quat']
        
        
        return {'pos': random_position, 'quat': random_orientation, 'parent': None}

    # Check if the node is in collision with any obstacles
    def is_in_collision(self, node):
        for obstacle in self.obstacle_list:
            if len(obstacle) == 4:  # Sphere: (x, y, z, radius)
                ox, oy, oz, radius = obstacle
                if np.linalg.norm(np.array([ox, oy, oz]) - node['pos']) <= radius:
                    return True
            elif len(obstacle) == 6 and obstacle[-1] == "desk":  # Desk: (x, y, z, width, height, "desk")
                x, y, z, width, height = obstacle[:-1]
                if (x <= node['pos'][0] / 2 and
                    y - width / 2 <= node['pos'][1] <= y + width/ 2 and
                    node['pos'][2] <= z + height):
                    return True
        return False

    # Get the index of the nearest node to the random point
    def get_nearest_node_index(self, random_point):
        distances = [self.distance(node, random_point) for node in self.node_list]
        return distances.index(min(distances))

    # Steer the node towards the random point
    def steer(self, from_node, to_point):
        direction = to_point['pos'] - from_node['pos']
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        new_position = from_node['pos'] + direction

        # Spherical Linear Interpolation (SLERP) for quaternion orientation
        start_rot = R.from_quat(from_node['quat'])
        goal_rot = R.from_quat(to_point['quat'])
        
        # Use the Slerp class to interpolate between the two orientations
        times = [0, 1]  # Start and end for interpolation
        slerp = Slerp(times, R.from_quat([from_node['quat'], to_point['quat']]))
        
        # Interpolate to the step size fraction
        slerp_fraction = min(self.step_size / distance, 1.0)
        new_orientation = slerp([slerp_fraction]).as_quat()[0]
        
        return {'pos': new_position, 'quat': new_orientation, 'parent': from_node}

    def find_path(self):
        min_dist_to_goal = float('inf')
        for _ in range(self.max_iter):
            random_point = self.get_random_point()
            nearest_index = self.get_nearest_node_index(random_point)
            nearest_node = self.node_list[nearest_index]

            new_node = self.steer(nearest_node, random_point)

            if not self.is_in_collision(new_node):
                self.node_list.append(new_node)

                pos_dist = np.linalg.norm(new_node['pos'] - self.goal['pos'])
                quat1 = R.from_quat(new_node['quat'])
                quat2 = R.from_quat(self.goal['quat'])
                rot_dist = quat1.inv() * quat2
                angle_diff = np.linalg.norm(rot_dist.as_rotvec())

                dist_to_goal = self.distance(new_node, self.goal)
                if dist_to_goal < min_dist_to_goal:
                    min_dist_to_goal = dist_to_goal
                # print(f"Positional distance: {pos_dist:.4f}, Angular distance: {angle_diff:.4f}, "
                #       f"Total distance: {dist_to_goal:.4f}")
                    #rospy.loginfo(f"RRT path at node {len(self.node_list)}: {self.build_path(new_node)}")

                if dist_to_goal < 0.01:
                    print("Goal reached!")
                    return self.build_path(new_node)
        #print(f"Final minimal distance to goal: {min_dist_to_goal:.4f}")
        return None

    # Build the path by traversing the nodes from the goal to the start, add the goal pose if not included
    def build_path(self, node):
        path = []
        while node is not None:
            path.append(np.concatenate((node['pos'], node['quat'])))
            node = node['parent']

        path = path[::-1]

        # Ensure the final goal pose is included
        if not np.allclose(path[-1][:3], self.goal['pos'], atol=1e-3) or \
        not np.allclose(path[-1][3:], self.goal['quat'], atol=1e-3):
            path.append(np.concatenate((self.goal['pos'], self.goal['quat'])))
        
        return path

    # Plot the trajectory in 3D space and save the plot to a file
    def plot_trajectory(self, path, save_path=None):
        """
        Plot the trajectory in 3D space.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the path
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], '-o', label='Trajectory')

        # Plot the start and goal
        ax.scatter(self.start['pos'][0], self.start['pos'][1], self.start['pos'][2], c='green', s=100, label='Start')
        ax.scatter(self.goal['pos'][0], self.goal['pos'][1], self.goal['pos'][2], c='red', s=100, label='Goal')


        # Set labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()

        if save_path:
            plt.savefig(save_path)
            print(f"Plot saved to {save_path}")

        plt.show(block=False)
        plt.pause(0.1)
    

def calculate_map_bounds(start, goal, margin=0.15):
    """
    Calculate map bounds based on start and goal positions with a margin.
    """
    start_pos = np.array(start[:3])  # [x, y, z] of start
    goal_pos = np.array(goal[:3])  # [x, y, z] of goal

    # Calculate the min and max bounds for each axis
    x_min = min(start_pos[0], goal_pos[0]) - margin
    x_max = max(start_pos[0], goal_pos[0]) + margin
    y_min = min(start_pos[1], goal_pos[1]) - margin
    y_max = max(start_pos[1], goal_pos[1]) + margin
    z_min = min(start_pos[2], goal_pos[2]) - margin
    z_max = max(start_pos[2], goal_pos[2]) + margin

    return [[x_min, x_max], [y_min, y_max], [z_min, z_max]]


