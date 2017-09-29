from Edge import RelativePose2D
from Node import Pose2D
from Graph import DirectedGraph
from IterativeSmoother import IterativeSmoother2D

graph = DirectedGraph()
node1 = Pose2D(0, 0, 0)
node2 = Pose2D(1.1, 0, 0)
node3 = Pose2D(1, 0, 0)
constraint12 = RelativePose2D(node1, node2, 1, 0, 0, [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
constraint23 = RelativePose2D(node2, node3, 1, 0, 0, [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
graph.add_node(node1)
graph.add_node(node2)
graph.add_node(node3)
graph.add_edge(constraint12)
graph.add_edge(constraint23)

optimizer = IterativeSmoother2D(gain=0.01, max_steps=100, debug=True)
optimizer.optimize(graph, node2)

for node in graph.nodes:
    print(node.value)
