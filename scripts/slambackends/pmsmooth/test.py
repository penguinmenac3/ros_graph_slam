import math

from Edge import RelativePose2D, UnaryEdge
from Node import Pose2D
from Graph import DirectedGraph
from IterativeSmoother import IterativeSmoother2D
import pose2d
from pose2d import tuple_to_numpy_pose as to_numpy

# First we need a graph where to store everything.
graph = DirectedGraph()

# Our poses need initial estimates which can be anything, however good estimates improve convergence speed.
node1 = Pose2D(0, 0, 0)
node2 = Pose2D(0, 0, 0)
node3 = Pose2D(0, 0, 0)

graph.add_node(node1)
graph.add_node(node2)
graph.add_node(node3)

# These are relative measurements (e.g. odometry).
constraint12 = RelativePose2D(node1, node2, 1, 0, math.radians(90), [[0.01, 0, 0], [0, 0.01, 0], [0, 0, math.radians(5)**2]])
constraint23 = RelativePose2D(node2, node3, 1, 0, 0, [[0.01, 0, 0], [0, 0.01, 0], [0, 0, math.radians(5)**2]])

graph.add_edge(constraint12)
graph.add_edge(constraint23)

# This is an external measurement (e.g. a marker in the environment)
constraint01 = UnaryEdge(node1, 0, 0, 0, [[0.0001, 0, 0], [0, 0.0001, 0], [0, 0, math.radians(1)**2]])

graph.add_edge(constraint01)

# Now we define our optimizer. The most important thing is to tell when to stop optimizing.
min_error_cutoff = 0.001
optimizer = IterativeSmoother2D(min_steps=1, max_steps=100,
                                min_contribution_cutoff=min_error_cutoff,
                                min_error_cutoff=min_error_cutoff,
                                debug=True)

# Here we optimize in a loop of two times.
# Note that the first time we try to optimize we optimize for 10 steps till we converge.
# The second time it converges after 1 step. This is essentially checking if the result still meets the error criteria.
err = 1
for i in range(2):
    # Provide some optical guideline for reading the log.
    print("######################################")
    print("Iteration %d" % i)

    # Optimize and print the error.
    err, steps = optimizer.optimize(graph, node3)
    print("Error: %.4f  |  Steps: %d" % (err, steps))

    # Print the estimated poses.
    print("Results:")
    for node in graph.nodes:
        print(pose2d.pretty_string(to_numpy(node.value)))

# The code works, if the optimizer converges with a low enough error rate.
print("")
print("Test Result:")
if err < min_error_cutoff:
    print("Test passed! Graph Optimizer converges.")
else:
    print("FAILED! The error is %.4f too high!" % (err - min_error_cutoff))