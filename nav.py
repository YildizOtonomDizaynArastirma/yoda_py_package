import json
import rclpy
from rclpy.node import Node
import numpy as np

class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.graph = None  # Initialize the graph
        self.current_node = 0
        self.read_file("roadmap.geojson")
        # self.get_logger().info(f"BFS path from 0 to 9: {self.bfs(0, 9)}") 
        self.get_logger().info(self.get_path(9))  # Example usage of get_path method

    def read_file(self, file_path, directed=False):
        try:
            with open(file_path, "r") as file:
                content = file.read()
                json_content = json.loads(content)

                # Extract features and build the graph
                features = json_content['features']
                num_nodes = len(features)
                self.graph = np.zeros((num_nodes, num_nodes), dtype=int)  # Adjacency matrix

                for i, feature in enumerate(features):
                    if 'adjacents' in feature['properties']:
                        adjacents = feature['properties']['adjacents']
                        for direction, neighbor in adjacents.items():
                            try:
                                neighbor_index = int(neighbor)  # Convert to integer
                                self.graph[i, neighbor_index] = 1
                                if not directed:
                                    self.graph[neighbor_index, i] = 1  # For undirected graph
                            except ValueError:
                                self.get_logger().warn(f"Invalid neighbor index: {neighbor}")
                            except IndexError:
                                self.get_logger().warn(f"Neighbor index out of range: {neighbor}")

        except FileNotFoundError:
            self.get_logger().error("File not found.")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def bfs(self, start, goal):
        if self.graph is None:
            self.get_logger().error("Graph not initialized. Call read_file() first.")
            return None

        num_nodes = self.graph.shape[0]
        visited = [False] * num_nodes
        queue = [(start, [start])]  # Store node and path
        visited[start] = True
        
        queue_index = 0 # Index to track the "front" of the queue

        while queue_index < len(queue): # Loop until the queue is empty
            node, path = queue[queue_index] # Get the element at the front
            queue_index += 1 # Increment the queue_index (dequeue)

            if node == goal:
                return path

            for neighbor in range(num_nodes):
                if self.graph[node, neighbor] == 1 and not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append((neighbor, path + [neighbor]))

        return None  # No path found

    def get_path(self, target_node):
        if self.graph is None:
            self.get_logger().error("Graph not initialized. Call read_file() first.")
            return None

        path = self.bfs(self.current_node, target_node)
        if path is not None:
            self.get_logger().info(f"Path found: {path}")
            return path
        else:
            self.get_logger().info("No path found.")
            return None
    
    def remove_edge(self, node1, node2, directed=False):
        if self.graph is None:
            self.get_logger().error("Graph not initialized. Call read_file() first.")
            return False

        if 0 <= node1 < self.graph.shape[0] and 0 <= node2 < self.graph.shape[0]:
            self.graph[node1, node2] = 0
            if not directed:
                self.graph[node2, node1] = 0
            self.get_logger().info(f"Edge removed between {node1} and {node2}.")
            return True
        else:
            self.get_logger().error("Invalid node indices.")
            return False    

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()