import rclpy
from rclpy.node import Node

class BidirectionalSearchNode(Node):
    def __init__(self):
        super().__init__('bidirectional_search')
        self.get_logger().info("Bidirectional Search Node Started")

    def bidirectional_search(self, graph, start, goal):
        if start == goal:
            return [start]

        forward_queue = {start}
        backward_queue = {goal}
        forward_visited = {start: None}
        backward_visited = {goal: None}

        while forward_queue and backward_queue:
            if intersection := forward_queue.intersection(backward_queue):
                return self.construct_path(intersection.pop(), forward_visited, backward_visited)

            forward_queue = self.expand_nodes(forward_queue, forward_visited, graph)
            backward_queue = self.expand_nodes(backward_queue, backward_visited, graph)

        return None

    def expand_nodes(self, queue, visited, graph):
        next_level = set()
        for node in queue:
            for neighbor in graph.get(node, []):
                if neighbor not in visited:
                    visited[neighbor] = node
                    next_level.add(neighbor)
        return next_level

    def construct_path(self, intersection, forward_visited, backward_visited):
        path = []
        node = intersection
        while node:
            path.append(node)
            node = forward_visited[node]
        path.reverse()

        node = backward_visited[intersection]
        while node:
            path.append(node)
            node = backward_visited[node]

        return path

def main(args=None):
    rclpy.init(args=args)
    node = BidirectionalSearchNode()

    # Example graph
    graph = {
        'A': ['B', 'C'],
        'B': ['A', 'D', 'E'],
        'C': ['A', 'F', 'G'],
        'D': ['B'],
        'E': ['B', 'H'],
        'F': ['C'],
        'G': ['C'],
        'H': ['E']
    }

    start, goal = 'A', 'H'
    path = node.bidirectional_search(graph, start, goal)
    node.get_logger().info(f"Path from {start} to {goal}: {path}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
