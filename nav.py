import json
import rclpy
from rclpy.node import Node
import numpy as np

class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.graph = None  # Adjacency matrix
        self.forced_transition = [[7, 11], [15, 14], [8, 4], [4, 16]]
        self.banned_transition = [[2, 3], [13, 12]]
        self.read_file()
        path = self.bfs(0, 16)
        self.get_logger().info(f"BFS path from 0 to 16 with constraints: {path}") 

    def read_file(self):
        try:
            with open("roadmap.geojson", "r") as file:
                content = file.read()
                json_content = json.loads(content)

                # Extract features and build the graph
                features = json_content['features']
                num_nodes = len(features)
                self.graph = np.zeros((num_nodes, num_nodes), dtype=int)

                for i, feature in enumerate(features):
                    if 'adjacents' in feature['properties']:
                        adjacents = feature['properties']['adjacents']
                        for direction, neighbor in adjacents.items():
                            try:
                                neighbor_index = int(neighbor)
                                self.graph[i, neighbor_index] = 1
                                self.graph[neighbor_index, i] = 1  # Undirected graph
                            except ValueError:
                                self.get_logger().warn(f"Invalid neighbor index: {neighbor}")
                            except IndexError:
                                self.get_logger().warn(f"Neighbor index out of range: {neighbor}")

        except FileNotFoundError:
            self.get_logger().error("File not found.")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def bfs(self, start, goal, max_visits=2):
        if self.graph is None:
            self.get_logger().error("Graph not initialized.")
            return None

        num_nodes = self.graph.shape[0]
        queue = [(start, [start])]  # (current_node, path)

        while queue:
            node, path = queue.pop(0)

            # Check banned transitions anywhere in path
            if self.path_has_banned(path):
                continue

            if node == goal:
                if self.path_has_forced(path):
                    return path  # Return first valid path
                continue

            for neighbor in range(num_nodes):
                if self.graph[node, neighbor] == 1:
                    # Prevent immediate reversal (no going back to previous node)
                    if len(path) >= 2 and neighbor == path[-2]:
                        continue

                    # Count visits to neighbor in the path
                    visits = path.count(neighbor)
                    if visits >= max_visits:
                        continue

                    queue.append((neighbor, path + [neighbor]))

        return None

    def path_has_forced(self, path):
        """Ensure all forced transitions are in the path."""
        for u, v in self.forced_transition:
            found = False
            for i in range(len(path) - 1):
                if path[i] == u and path[i+1] == v:
                    found = True
                    break
            if not found:
                return False
        return True

    def path_has_banned(self, path):
        """Check if any banned transitions are in the path."""
        for u, v in self.banned_transition:
            for i in range(len(path) - 1):
                if path[i] == u and path[i+1] == v:
                    return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
