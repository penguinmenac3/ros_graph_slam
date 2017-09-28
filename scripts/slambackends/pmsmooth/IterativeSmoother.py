import math
import pose2d
import collections


class IterativeSmoother2D(object):
    def __init__(self, max_steps=100, min_steps=10, gain=0.5, min_contribution_cutoff=0.01, min_error_cutoff=0.01):
        self.max_steps = max_steps
        self.min_steps = min_steps
        self.gain = gain
        self.min_contribution_cutoff = min_contribution_cutoff
        self.min_error_cutoff = min_error_cutoff

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

                # Calculate gradient
                delta = self.neg_delta_error(graph, node)

                # Update
                delta = [self.gain * d for d in delta]
                node.values = pose2d.add(node.values, delta)

                # Add connected nodes for updates
                visited.append(node)
                for edge in graph.edges[node]:
                    if edge.from_node == node and edge.to_node not in visited and edge.to_node not in to_visit:
                        to_visit.append(edge.to_node)
                    if edge.to_node == node and edge.from_node not in visited and edge.from_node not in to_visit:
                        to_visit.append(edge.from_node)

            error = self.error(graph, start_node)
        return error

    def error(self, graph, node):
        error = 0
        for edge in graph.edges[node]:
            error += edge.error(node)

        return error

    def neg_delta_error(self, graph, node):
        error = [0, 0, 0, 0]
        n = float(len(graph.edges[node]))
        for edge in graph.edges[node]:
            t = edge.neg_delta_error(node)

            error[0] += t[0] / n
            error[1] += t[1] / n
            error[2] += math.cos(t[2])
            error[3] += math.sin(t[2])

        return error[0], error[1], math.atan2(error[3], error[2])
