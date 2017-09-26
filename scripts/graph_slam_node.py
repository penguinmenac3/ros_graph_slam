#!/usr/bin/env python
"""
    A graph slam ros node.
"""
import threading
import sys
import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, PointCloud2, LaserScan
from slambackends.factorgraph import FactorGraph
from geometry_msgs.msg import Point, Quaternion, Pose2D
from ros_graph_slam.msg import PoseNode, BetweenFactor, Path2D

def length_of_pose(pose):
    '''
        Calculate the length of a pose.
    '''
    return math.sqrt(pose[0] * pose[0] + pose[1] * pose[1])

def relative_pose(pose1, pose2):
    '''
        Calculate the relative pose between two poses.
    '''
    x = pose2[0] - pose1[0]
    y = pose2[1] - pose1[1]

    dtheta = pose2[2] - pose1[2]

    delta_x = math.cos(-pose1[2]) * x - math.sin(-pose1[2]) * y
    delta_y = math.sin(-pose1[2]) * x + math.cos(-pose1[2]) * y

    return [delta_x, delta_y, dtheta]

class GraphSLAM(object):
    """
        A graph slam to solve the loop closure slam problem.
    """

    tf_br = tf.TransformBroadcaster()
    publish_tf = True

    def __init__(self, map_path=None):
        self.lock = threading.Lock()
        self.map_lon = None
        self.map_lat = None
        self.last_scan = None
        self.last_laserscan = None
        self.last_idx = {}
        self.last_pose = {"main_pose": [0, 0, 0]}
        self.graph = FactorGraph(map_path)
        self.last_ori = (0, 0, 0, 1)
        self.last_pos = (0, 0, 0)
        self.last_odom_pose = (0, 0, 0)
        self.relocalization_required = False

        if map_path is not None:
            self.relocalization_required = True

        self.min_covariance_pose = [[0.01 * 0.01, 1e-8, 1e-8],
                            [1e-8, 0.01 * 0.01, 1e-8],
                            [1e-8, 1e-8, math.radians(2) * math.radians(2)]]

        self.min_covariance_main_pose = [[0.01 * 0.01, 1e-8, 1e-8],
                            [1e-8, 0.01 * 0.01, 1e-8],
                            [1e-8, 1e-8, math.radians(10) * math.radians(10)]]

        self.min_covariance_between_factor = [[0.03 * 0.03, 1e-8, 1e-8],
                            [1e-8, 0.03* 0.03, 1e-8],
                            [1e-8, 1e-8, math.radians(5) * math.radians(5)]]

        rospy.init_node('graph_slam')

        main_pose = "odom"
        between_topic = "between_factor"
        poses = "scanmatch"
        navsat = "gps"
        if rospy.has_param('odometry'):
            main_pose = rospy.get_param("odometry")
        if rospy.has_param('poses'):
            poses = rospy.get_param("poses")
        if rospy.has_param('navsat'):
            navsat = rospy.get_param("navsat")

        self.path_pub = rospy.Publisher('trajectory', Path, queue_size=1)
        self.path_2d_pub = rospy.Publisher('trajectory2d', Path2D, queue_size=1)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
        self.pose_node_pub = rospy.Publisher('pose_node', PoseNode, queue_size=1)

        rospy.Subscriber(main_pose, Odometry, self.main_pose_callack)
        rospy.Subscriber(between_topic, BetweenFactor, self.between_callback)

        rospy.Subscriber("scan", PointCloud2, self.scan_callback)
        rospy.Subscriber("laserscan", LaserScan, self.laserscan_callback)

        for pose in poses.split(","):
            rospy.Subscriber(pose, PoseWithCovarianceStamped, lambda x: self.pose_callack(pose, x))

        for navsat in navsat.split(","):
            rospy.Subscriber(navsat, NavSatFix, self.navsat_callback)

        if map_path is not None:
            self.publish(self.graph.initial_poses, rospy.get_time())

    def save(self, map_path):
        self.graph.save(map_path)

    def publish(self, result, time_stamp):
        curPath = Path()
        curPath2D = Path2D()
        curPath.header.frame_id = "map"
        curPath2D.header.frame_id = "map"
        lastPose = None
        for x in result:
            p2 = Pose2D()
            p2.x = x[0]
            p2.y = x[1]
            p2.theta = x[2]
            curPath2D.poses.append(p2)
            p = PoseStamped()
            p.header.stamp = time_stamp
            p.header.frame_id = "map"
            p.pose.position.x = x[0]
            p.pose.position.y = x[1]
            p.pose.position.z = 0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, x[2])
            p.pose.orientation = Quaternion(*quaternion)
            curPath.poses.append(p)
            lastPose = p
            lastMapPose = x

        self.path_pub.publish(curPath)
        self.path_2d_pub.publish(curPath2D)
        if lastPose is not None:
            self.pose_pub.publish(lastPose)

        p = PoseStamped()
        delta = lastMapPose[2] - self.last_odom_pose[2]
        c = math.cos(delta)
        s = math.sin(delta)
        #px = c * lastMapPose[0] - s * lastMapPose[1]
        #py = s * lastMapPose[0] + c * lastMapPose[1]
        px = lastMapPose[0]
        py = lastMapPose[1]
        lx = c * self.last_odom_pose[0] - s * self.last_odom_pose[1]
        ly = s * self.last_odom_pose[0] + c * self.last_odom_pose[1]
        p.header.stamp = time_stamp
        p.header.frame_id = "map"
        p.pose.position.x = px - lx
        p.pose.position.y = py - ly
        p.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, delta)
        p.pose.orientation = Quaternion(*quaternion)

        pos = (p.pose.position.x,
               p.pose.position.y,
               p.pose.position.z)

        ori = (p.pose.orientation.x,
               p.pose.orientation.y,
               p.pose.orientation.z,
               p.pose.orientation.w)

        # Also publish tf if necessary
        if self.publish_tf:
            self.tf_br.sendTransform(pos, ori, time_stamp, "/odom", "/map")
            self.last_ori = ori
            self.last_pos = pos

    def scan_callback(self, data):
        self.last_scan = data

    def laserscan_callback(self, data):
        self.last_laserscan = data

    def main_pose_callack(self, data):
        if self.relocalization_required:
            return
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        pose = [0, 0, 0]
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y
        pose[2] = tf.transformations.euler_from_quaternion(quaternion)[2]
        last_pose = self.last_pose["main_pose"]

        rel_pose = relative_pose(last_pose, pose)


        if self.publish_tf and self.last_ori is not None and self.last_pos is not None:
            self.tf_br.sendTransform(self.last_pos, self.last_ori, data.header.stamp, "/odom", "/map")

        if length_of_pose(rel_pose) < 0.3:
            return

        self.last_pose["main_pose"] = pose

        covariance = [[data.pose.covariance[0], data.pose.covariance[1], data.pose.covariance[5]],
                      [data.pose.covariance[6], data.pose.covariance[7], data.pose.covariance[11]],
                      [data.pose.covariance[30], data.pose.covariance[31], data.pose.covariance[35]]]

        for row in range(len(covariance)):
            for col in range(len(covariance[row])):
                covariance[row][col] = max(covariance[row][col], self.min_covariance_main_pose[row][col])

        if self.last_scan is not None and self.last_laserscan is not None:
            pose_node = PoseNode()
            pose_node.id = self.graph.get_current_index()
            pose_node.scan = self.last_scan
            pose_node.laserscan = self.last_laserscan
            self.pose_node_pub.publish(pose_node)

        self.last_odom_pose = pose

        self.lock.acquire()
        self.graph.append_relative_pose(rel_pose, covariance)

        result = self.graph.optimize()
        self.lock.release()
        self.publish(result, data.header.stamp)

    def pose_callack(self, topic, data):
        if self.relocalization_required:
            return
        current_idx = self.graph.get_current_index()
        if not topic in self.last_idx:
            self.last_idx[topic] = current_idx
            self.last_pose[topic] = self.graph.get_current_estimate()[-1]
            return
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        pose = [0, 0, 0]
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y
        pose[2] = tf.transformations.euler_from_quaternion(quaternion)[2]
        last_pose = self.last_pose[topic]

        rel_pose = relative_pose(last_pose, pose)
        if length_of_pose(rel_pose) < 0.2:
            return

        last_idx = self.last_idx[topic]

        if last_idx == current_idx:
            return

        self.last_idx[topic] = current_idx
        self.last_pose[topic] = pose

        covariance = [[data.pose.covariance[0], data.pose.covariance[1], data.pose.covariance[5]],
                      [data.pose.covariance[6], data.pose.covariance[7], data.pose.covariance[11]],
                      [data.pose.covariance[30], data.pose.covariance[31], data.pose.covariance[35]]]

        for row in range(len(covariance)):
            for col in range(len(covariance[row])):
                covariance[row][col] = max(covariance[row][col], self.min_covariance_pose[row][col])

        self.lock.acquire()
        self.graph.insert_relative_pose(last_idx, current_idx, rel_pose, covariance)
        self.lock.release()

    def between_callback(self, data):
        """
            Accept a between factor.
        """
        rel_pose = [data.pose.x, data.pose.y, data.pose.theta]
        last_idx = data.from_id
        current_idx = data.to_id

        covariance = [[data.covariance[0], data.covariance[1], data.covariance[2]],
                      [data.covariance[3], data.covariance[4], data.covariance[5]],
                      [data.covariance[6], data.covariance[7], data.covariance[8]]]

        for row in range(len(covariance)):
            for col in range(len(covariance[row])):
                covariance[row][col] = max(covariance[row][col], self.min_covariance_between_factor[row][col])

        self.lock.acquire()
        if self.relocalization_required and current_idx == self.graph.get_current_index() + 1:
            self.graph.append_relative_pose(rel_pose, covariance, last_idx)
            self.relocalization_required = False
        else:
            self.graph.insert_relative_pose(last_idx, current_idx, rel_pose, covariance)
        result = self.graph.optimize(10)
        self.lock.release()

        self.publish(result, data.header.stamp)

    def navsat_callback(self, data):
        if self.map_lon is None:
            self.map_lon = data.longitude
        if self.map_lat is None:
            self.map_lat = data.latitude

        # TODO
        y = (data.latitude - self.map_lat)
        x = (data.longitude - self.map_lon)
        acc = 10

        self.lock.acquire()
        self.graph.add_absolute_position(self.graph.get_current_index(), (x, y), acc)
        self.lock.release()


def main():
    """
        The main function.

        It parses the arguments and sets up graph slam.

        Usage: python graph_slam_node.py /path/to/load/map.npz /path/to/save/map.npz
        or
        Usage: python graph_slam_node.py /path/to/load/map.npz
        or
        Usage: python graph_slam_node.py None /path/to/save/map.npz
        or
        Usage: python graph_slam_node.py
    """
    map_input_path = None
    if len(sys.argv) > 1:
        map_input_path = sys.argv[1]
        if map_input_path == "None":
            map_input_path = None
    slam = GraphSLAM(map_input_path)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    if len(sys.argv) > 2:
        slam.save(sys.argv[2])
    sys.exit(0)


if __name__ == "__main__":
    main()
