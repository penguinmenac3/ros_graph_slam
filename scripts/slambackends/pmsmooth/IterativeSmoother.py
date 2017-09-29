import math
import pose2d
import numpy as np
import collections


class IterativeSmoother2D(object):
    def __init__(self, max_steps=100, min_steps=1, min_contribution_cutoff=0.01, min_error_cutoff=0.01,
                 debug=False):
        self.max_steps = max_steps
        self.min_steps = min_steps
        self.min_contribution_cutoff = min_contribution_cutoff
        self.min_error_cutoff = min_error_cutoff
        self.debug = debug

    def optimize(self, graph, start_node):
        i = 0
        error = 0
        while (i < self.min_steps or error > self.min_error_cutoff) and i < self.max_steps:
            i += 1
            to_visit = collections.deque()
            to_visit.append(start_node)
            visited = [None]

            while len(to_visit) > 0:
                node = to_visit.popleft()

                # Calculate error to decide if we need to update
                if self.error(graph, node) < self.min_contribution_cutoff:
                    continue

                # Update
                if self.debug:
                    print("**********************")
                    print("   " + pose2d.pretty_string(pose2d.tuple_to_numpy_pose(node.value)))
                node.value = self.estimate_node_pose(graph, node)
                if self.debug:
                    print("-> " + pose2d.pretty_string(pose2d.tuple_to_numpy_pose(node.value)))

                # Add connected nodes for updates
                visited.append(node)
                for edge in graph.edges[node]:
                    if edge.from_node == node and edge.to_node not in visited and edge.to_node not in to_visit:
                        to_visit.append(edge.to_node)
                    if edge.to_node == node and edge.from_node not in visited and edge.from_node not in to_visit:
                        to_visit.append(edge.from_node)

            error = self.error(graph, start_node)
        return error, i

    def error(self, graph, node):
        error = 0
        for edge in graph.edges[node]:
            error += edge.error(node)

        return error

    def estimate_node_pose(self, graph, node):
        mean = [0, 0, 0, 0]
        mean_norm = [0, 0]
        for edge in graph.edges[node]:
            q = edge.estimate_pose(node)
            if self.debug:
                print("q: " + pose2d.pretty_string(q))
            t = np.dot(edge.weight, q)
            n = np.dot(edge.weight, np.array([[1], [1], [1]]))

            mean_norm[0] += n[0][0]
            mean_norm[1] += n[1][0]
            mean[0] = mean[0] + t[0][0]
            mean[1] = mean[1] + t[1][0]
            mean[2] = mean[2] + math.cos(q[2][0]) * edge.weight[2][2]
            mean[3] = mean[3] + math.sin(q[2][0]) * edge.weight[2][2]

        return mean[0] / mean_norm[0], mean[1] / mean_norm[1], math.atan2(mean[3], mean[2])
