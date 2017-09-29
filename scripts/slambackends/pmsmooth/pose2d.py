from math import cos, sin, degrees
import numpy as np


def pretty_string(pose):
    """
    Convert a pose into a printable human readable string.
    :param pose: np.array containing 2d Pose [[x], [y], [theta]]
    :return: A string
    """
    x, y, theta = numpy_pose_to_tuple(pose)
    return "(%.2f, %.2f, %.2f)" % (x, y, degrees(theta))


def rotation_matrix(theta):
    """
    Create a rotation matrix for poses using theta
    :param theta: The rotationg angle in radians.
    :return: A numpy 2d array (3x3).
    """

    return np.array([[cos(theta), -sin(theta), 0],
                     [sin(theta), cos(theta), 0],
                     [0, 0, 1]])


def tuple_to_numpy_pose(pose):
    """
    Converts a pose given as tuple to numpy format.
    :param pose: Tuple containing 2d pose (x, y, theta)
    :return: np.array containing 2d Pose [[x], [y], [theta]]
    """
    x, y, theta = pose
    return np.array([[x], [y], [theta]])


def numpy_pose_to_tuple(pose):
    """
    Converts a pose given as numpy format to tuple.
    :param pose: np.array containing 2d Pose [[x], [y], [theta]]
    :return: Tuple containing 2d pose (x, y, theta)
    """
    return pose[0][0], pose[1][0], pose[2][0]


def calculate_relative_pose(from_pose, to_pose):
    """
    Calculate the relative pose.
    :param from_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :param to_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :return: np.array containing 2d Pose [[x], [y], [theta]]
    """

    return np.dot(rotation_matrix(from_pose[2][0]).transpose(), to_pose - from_pose)


def calculate_previous_pose(to_pose, relative_pose):
    """
    Calculate the pose from which to start to reach to_pose given a relative pose.
    :param to_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :param relative_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :return: np.array containing 2d Pose [[x], [y], [theta]]
    """

    return to_pose - np.dot(rotation_matrix(to_pose[2][0]-relative_pose[2][0]), relative_pose)


def calculate_next_pose(from_pose, relative_pose):
    """
    Calculate the pose that is reached when starting from a pose and appending a relative pose.
    :param from_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :param relative_pose: np.array containing 2d Pose [[x], [y], [theta]]
    :return: np.array containing 2d Pose [[x], [y], [theta]]
    """

    return from_pose + np.dot(rotation_matrix(from_pose[2][0]), relative_pose)
