class Node(object):
    def __init__(self, value):
        self.value = value


class Pose2D(Node):
    def __init__(self, x, y, theta):
        super(Pose2D, self).__init__((x, y, theta))
