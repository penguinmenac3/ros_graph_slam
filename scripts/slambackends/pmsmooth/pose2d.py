from math import cos, sin
import numpy as np


def subtract(a, b):
    """
    Calculates a - b on poses.
    :param a: Tuple containing 2d Pose (x, y, theta)
    :param b: Tuple containing 2d Pose (x, y, theta)
    :return: Tuple containing 2d Pose (x, y, theta)
    """
    a_x, a_y, a_theta = a
    b_x, b_y, b_theta = b

    r_theta = a_theta - b_theta
    r_x = a_x - b_x
    r_y = a_y - b_y

    t_x = cos(-b_theta) * r_x - sin(-b_theta) * r_y
    t_y = sin(-b_theta) * r_x + cos(-b_theta) * r_y
    t_theta = r_theta

    return t_x, t_y, t_theta


def delta_sub_a(a, b):
    """
    Calculate the derivative of sub with respect to a.
    :param a: Tuple containing 2d Pose (x, y, theta)
    :param b: Tuple containing 2d Pose (x, y, theta)
    :return: Jaccobian with respect to a at a, b.
    """
    a_x, a_y, a_theta = a
    b_x, b_y, b_theta = b

    return np.array([[cos(-b_theta), -sin(-b_theta), 0],
                     [sin(-b_theta), cos(-b_theta), 0],
                     [0, 0, 1]])


def delta_sub_b(a, b):
    """
    Calculate the derivative of sub with respect to b.
    :param a: Tuple containing 2d Pose (x, y, theta)
    :param b: Tuple containing 2d Pose (x, y, theta)
    :return: Jaccobian with respect to b at a, b.
    """
    a_x, a_y, a_theta = a
    b_x, b_y, b_theta = b

    d1 = b_x * sin(-b_theta) + b_y * cos(-b_theta)
    d2 = -b_x * cos(-b_theta) + b_y * sin(-b_theta)

    return np.array([[-cos(-b_theta), +sin(-b_theta), -d1],
                     [-sin(-b_theta), -cos(-b_theta), -d2],
                     [0, 0, -1]])


def add(a, b):
    """
    Calculates a + b on poses.
    b is a relative pose that is added to a.
    :param a: Tuple containing 2d Pose (x, y, theta)
    :param b: Tuple containing 2d Pose (x, y, theta)
    :return: Tuple containing 2d Pose (x, y, theta)
    """
    a_x, a_y, a_theta = a
    b_x, b_y, b_theta = b

    r_theta = b_theta + a_theta
    r_x = a_x + cos(a_theta) * b_x - sin(a_theta) * b_y
    r_y = a_y + sin(a_theta) * b_x + cos(a_theta) * b_y

    return r_x, r_y, r_theta