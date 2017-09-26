"""
    A factorgraph implementation to optimize a pose graph.
"""
from __future__ import print_function

from threading import Lock
import math
import numpy as np
from slambackends.gtsam import gtsam
from graphviz import Digraph


def normalize_pose(pose):
    """
        Normalizes the theta angle of a pose between 0 and 2 * Pi.
    """
    new_theta = pose[2]
    if new_theta > 2 * math.pi:
        new_theta -= 2 * math.pi
    if new_theta <= 0:
        new_theta += 2 * math.pi
    return [pose[0], pose[1], new_theta]

def add_relative_pose(absolute, relative):
    """
        Add a relative pose on an absolute.
    """
    new_theta = absolute[2] + relative[2]
    if new_theta > math.pi:
        new_theta -= 2 * math.pi
    if new_theta < -math.pi:
        new_theta += 2 * math.pi

    delta_x = math.cos(absolute[2]) * relative[0] - math.sin(absolute[2]) * relative[1]
    delta_y = math.sin(absolute[2]) * relative[0] + math.cos(absolute[2]) * relative[1]

    return normalize_pose([absolute[0] + delta_x, absolute[1] + delta_y, new_theta])


class FactorGraph(object):
    """
        A simple factor graph with no hierachy.
    """
    def __init__(self, load_file=None, debug=True):
        self.debug = debug
        self.lock = Lock()

        # All variables required for a persistent state of the graph or rebuilding it from memory.
        self.initial_poses = [[0, 0, 0]]
        self.relative_edges = []  # contains (from_id, to_id, pose, covariance)
        self.gps = {}

        # The working memory version of graph slam using gtsam.
        self.graph_backend = gtsam.NonlinearFactorGraph()
        self.graph_estimates = gtsam.Values()

        parameters = gtsam.ISAM2Params()
        #print(dir(parameters))
        parameters.setRelinearizeThreshold(0.0001)
        parameters.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(parameters)

        # Initialize the graph with the origin pose.
        self.graph_estimates.insert(1, gtsam.Pose2(self.initial_poses[0][0], self.initial_poses[0][1], self.initial_poses[0][2]))
        priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
        priorNoise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
        self.graph_backend.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise))

        # Load state from file if required
        if load_file is not None:
            data = dict(np.load(load_file))
            # Re-initialize nodes (poses)
            self.initial_poses = data["initial_poses"]
            for idx in range(len(self.initial_poses)):
                global_pos = self.initial_poses[idx]
                self.graph_estimates.insert(idx + 1, gtsam.Pose2(global_pos[0], global_pos[1], global_pos[2]))
            # Re-Initialize edges
            self.relative_edges = data["relative_edges"]
            for edge in self.relative_edges:
                from_id, to_id, pose, covariance = edge
                noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([covariance[0][0], covariance[1][1], covariance[2][2]]))
                self.graph_backend.add(gtsam.BetweenFactorPose2(from_id, to_id, gtsam.Pose2(pose[0], pose[1], pose[2]), noise))
            # Re-initialize gps
            self.gps = data["gps"]
            print("WARNING: GPS cannot be restored correctly yet.")

        # Also write the debug graph
        if self.debug:
            self.graph_debug = Digraph(comment="Debug factorgraph")
            self.graph_debug.node(str(1), "(" + str(1) + ") {:4.2f}".format(self.initial_poses[0][0]) + " {:4.2f}".format(self.initial_poses[0][1]) + " {:4.2f}".format(self.initial_poses[0][2]))

    def save(self, filenname):
        np.savez(filenname, initial_poses=self.initial_poses, relative_edges=self.relative_edges, gps=self.gps)

    def get_current_estimate(self):
        """
            Returns the current estimate of the trajectory without reoptimizing it.
        """
        return self.initial_poses

    def optimize(self, steps=1):
        """
            Optimize the graph.

            Returns the trajectory of the main_poses after optimization.
        """
        self.lock.acquire()
        print("Optimizing: " + str(self.get_current_index()))
        result = []
        try:
            # Update the graph
            self.isam.update(self.graph_backend, self.graph_estimates)
            for i in range(steps):
                self.isam.update()

            # Extract the estimates from the graph back into the initial_estimates.
            current_estimate = self.isam.calculateEstimate()
            for i in range(current_estimate.keys().size()):
                k = current_estimate.keys().at(i)
                pose = current_estimate.atPose2(k)
                result.append([pose.x(), pose.y(), pose.theta()])

            self.initial_poses = result

            # Reset everything for the next run
            self.graph_backend = gtsam.NonlinearFactorGraph()
            self.graph_estimates.clear()
        except:
            # Plot the graph that was unsolvable. This sometimes helps alot!
            self.graph_debug.render('error-graph.gv', view=True)
        self.lock.release()
        return result

    def append_relative_pose(self, pose, covariance, from_id=-1):
        """
            Adds a relative pose in the graph to the main datastream.
            This increments the index and those poses are used for initial estimates.

            The pose is expected as an array [x,y,theta].

            You must provide a valid covariance matrix for the pose as a 2d row first array.

            Returns if successfull.
        """
        self.lock.acquire()

        # Prepare helper variables
        pose = normalize_pose(pose)
        idx = self.get_current_index() + 1
        global_pos = add_relative_pose(self.initial_poses[from_id], pose)

        # Insert pose into initial_estimates
        self.initial_poses.append(global_pos)
        self.graph_estimates.insert(idx, gtsam.Pose2(global_pos[0], global_pos[1], global_pos[2]))
        if self.debug:
            self.graph_debug.node(str(idx), "(" + str(idx) + ") {:4.2f}".format(global_pos[0]) + " {:4.2f}".format(global_pos[1]) + " {:4.2f}".format(global_pos[2]))

        # Insert relative edge into graph.
        if from_id < 0:
            from_id = idx - 1
        state = self.insert_relative_pose(from_id, idx, pose, covariance, locked=True, prefix="o")
        if not state:
            print("ERROR")
        self.lock.release()
        return state

    def get_current_index(self):
        """
            Gets the current last index of the main datastream.
        """
        return len(self.initial_poses)

    def insert_relative_pose(self, from_id, to_id, pose, covariance, locked=False, prefix="s"):
        """
            Adds a relative pose to the graph.

            The provided id must be valid: 1 <= id <= self.get_current_index().

            The pose is expected as an array [x,y,theta].

            You must provide a valid covariance matrix for the pose as a 2d row first array.

            Returns if successfull.
        """
        pose = normalize_pose(pose)
        if not locked:
            self.lock.acquire()

        # Check if the id is valid.
        if (from_id < 1 or to_id < 1 or
                from_id > self.get_current_index() or
                to_id > self.get_current_index() or
                from_id == to_id):
            if not locked:
                self.lock.release()
            return False

        # Insert the node into the graph, if the id is valid.
        noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([covariance[0][0], covariance[1][1], covariance[2][2]]))
        self.graph_backend.add(gtsam.BetweenFactorPose2(from_id, to_id, gtsam.Pose2(pose[0], pose[1], pose[2]), noise))
        self.relative_edges.append((from_id, to_id, pose, covariance))
        if self.debug:
            self.graph_debug.edge(str(from_id), str(to_id), prefix + " {:4.2f}".format(pose[0]) + " {:4.2f}".format(pose[1]) + " {:4.2f}".format(pose[2]))

        if not locked:
            self.lock.release()
        return True

    def add_absolute_position(self, to_id, position, accuracy):
        """
            [NOT IMPLEMENTED YET]
            Adds an absolute constraint to the graph.

            It also applies feature transform to the current estimate to best explain the measurements.
        """
        self.lock.acquire()
        self.gps[to_id] = position

        # TODO add to graph

        if len(self.gps) >= 3:
            # Fit the initial poses to the global model.
            pass

        self.lock.release()
