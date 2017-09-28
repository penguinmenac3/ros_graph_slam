class DirectedGraph:
    def __init__(self):
        self.nodes = []
        self.edges = {}

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, edge):
        if edge.from_node is not None:
            if edge.from_node not in self.edges:
                self.edges[edge.from_node] = []
            self.edges[edge.from_node].append(edge)

        if edge.to_node is not None:
            if edge.to_node not in self.edges:
                self.edges[edge.to_node] = []
            self.edges[edge.to_node].append(edge)

    def remove_edge(self, edge):
        if edge.from_node is not None and edge.from_node in self.edges and edge in self.edges[edge.from_node]:
            self.edges[edge.from_node].remove(edge)

        if edge.to_node is not None and edge.to_node in self.edges and edge in self.edges[edge.to_node]:
            self.edges[edge.to_node].remove(edge)
