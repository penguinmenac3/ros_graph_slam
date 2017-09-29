import math
import numpy as np
import pose2d
from pose2d import tuple_to_numpy_pose as to_numpy


class Edge(object):
    def __init__(self, from_node, to_node, weight):
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight

    def error(self, node):
        raise NotImplementedError("Not implemented. Base class must implement this.")

    def estimate_pose(self, node):
        """
        Calculate the pose the node would have assuming it's neighbour was correct.
        :param node: The node that should be estimated as a node object.
        :return: The estimated position as np.array (3x1)
        """
        raise NotImplementedError("Not implemented. Base class must implement this.")


class UnaryEdge(Edge):
    def __init__(self, to_node, x, y, theta, covariance_matrix):
        super(UnaryEdge, self).__init__(None, to_node, np.linalg.inv(np.array(covariance_matrix)))
        self.abs_pose = (x, y, theta)

    def error(self, node):
        # Forward edge (normal direction)
        if node == self.to_node:
            x_i = to_numpy(node.value)
            x_j = to_numpy((0, 0, 0))

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

        # relative pose points from j to i
        r_ji = to_numpy(self.abs_pose)

        return self._loss(pose2d.calculate_relative_pose(pose2d.calculate_relative_pose(x_i, x_j), r_ji))

    def estimate_pose(self, node):
        if node == self.to_node:
            return pose2d.calculate_next_pose(to_numpy((0, 0, 0)), to_numpy(self.abs_pose))

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

    def _loss(self, pose):
        """
        Calculates the loss of an offset pose pose
        :param pose: Tuple containing 2d Pose (x, y, theta)
        :return: The norm as scalar value
        """
        t = np.dot(np.dot(pose.transpose(), self.weight), pose)
        return t[0][0]

class RelativePose2D(Edge):
    def __init__(self, from_node, to_node, x, y, theta, covariance_matrix):
        super(RelativePose2D, self).__init__(from_node, to_node, np.linalg.inv(np.array(covariance_matrix)))
        self.rel_pose = (x, y, theta)

    def error(self, node):
        delta = None
        # Forward edge (normal direction)
        if node == self.to_node:
            to_node = pose2d.calculate_next_pose(to_numpy(self.from_node.value), to_numpy(self.rel_pose))
            delta = pose2d.calculate_relative_pose(to_node, to_numpy(self.to_node.value))

        # Backward edge (backwards direction)
        elif node == self.from_node:
            from_node = pose2d.calculate_previous_pose(to_numpy(self.to_node.value), to_numpy(self.rel_pose))
            delta = pose2d.calculate_relative_pose(from_node, to_numpy(self.from_node.value))

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

        return self._loss(delta)

    def estimate_pose(self, node):
        if node == self.to_node:
            return pose2d.calculate_next_pose(to_numpy(self.from_node.value), to_numpy(self.rel_pose))

        # Backward edge (backwards direction)
        elif node == self.from_node:
            return pose2d.calculate_previous_pose(to_numpy(self.to_node.value), to_numpy(self.rel_pose))

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

    def _loss(self, pose):
        """
        Calculates the loss of an offset pose pose
        :param pose: Tuple containing 2d Pose (x, y, theta)
        :return: The norm as scalar value
        """
        t = np.dot(np.dot(pose.transpose(), self.weight), pose)
        return t[0][0]
