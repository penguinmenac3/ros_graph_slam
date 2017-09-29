import math
import numpy as np
import pose2d


class Edge(object):
    def __init__(self, from_node, to_node):
        self.from_node = from_node
        self.to_node = to_node

    def error(self, node):
        raise NotImplementedError("Not implemented. Base class must implement this.")

    def neg_delta_error(self, node):
        raise NotImplementedError("Not implemented. Base class must implement this.")

    def inverse(self):
        return self


class UnaryEdge(Edge):
    def __init__(self, to_node):
        super(UnaryEdge, self).__init__(None, to_node)


class RelativePose2D(Edge):
    def __init__(self, from_node, to_node, x, y, theta, covariance_matrix):
        super(RelativePose2D, self).__init__(from_node, to_node)
        self.rel_pose = (x, y, theta)
        self.inv_covariance = np.linalg.inv(np.array(covariance_matrix))

    def error(self, node):
        # Forward edge (normal direction)
        if node == self.to_node:
            x_i = node.value
            x_j = self.from_node.value

        # Backward edge (backwards direction)
        elif node == self.from_node:
            x_i = self.to_node.value
            x_j = node.value

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

        # relative pose points from j to i
        r_ji = self.rel_pose

        return self._loss(pose2d.subtract(pose2d.subtract(x_i, x_j), r_ji))

    def neg_delta_error(self, node):
        """
        The negative derivative of the error with respect to the node.

        Basically how to move the node to get the correct value.
        With covariance = 0 this return will make a - b = r. Eg assume node is a and we want to know what to add to a
        so that it is equal to b + r => a + return = b + r
        With covariance = infinity this value would be zero.
        :param node:
        :return:
        """
        term1 = None
        term2 = None
        term3 = None# Forward edge (normal direction)
        if node == self.to_node:
            x_i = node.value
            x_j = self.from_node.value

            term1 = pose2d.delta_sub_a(x_i, x_j)

        # Backward edge (backwards direction)
        elif node == self.from_node:
            x_i = self.to_node.value
            x_j = node.value

            term1 = pose2d.delta_sub_b(x_i, x_j)

        # Invalid case
        else:
            raise ValueError("The provided node is not part of the connection.")

        # relative pose points from j to i
        r_ji = self.rel_pose

        term2 = pose2d.delta_sub_a(pose2d.subtract(x_i, x_j), r_ji)
        term3 = self._delta_loss(pose2d.subtract(pose2d.subtract(x_i, x_j), r_ji))

        result = np.dot(term1, np.dot(term2, term3))

        # FIXME do I just need to invert the sign here for the negative gradient?
        return -result[0][0], -result[1][0], -result[2][0]

    def _loss(self, pose):
        """
        Calculates the loss of an offset pose pose
        :param pose: Tuple containing 2d Pose (x, y, theta)
        :return: The norm as scalar value
        """
        x, y, theta = pose
        p = np.array([[x, y, theta]])
        p_t = np.array([[x], [y], [theta]])
        c = self.inv_covariance

        # pose * inv_covariance * pose^T
        return np.dot(np.dot(p, c), p_t)

    def _delta_loss(self, pose):
        """
        Calculate the derivative of the loss with respect to the pose.
        :param pose: Tuple containing 2d Pose (x, y, theta)
        :return: A numpy vector containing the derivative.
        """
        x, y, theta = pose
        c = self.inv_covariance
        c_t = self.inv_covariance.transpose()
        return np.dot((c + c_t), np.array([[x], [y], [theta]]))
