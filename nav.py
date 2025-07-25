import json
import rclpy
from rclpy.node import Node
import numpy as np

class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.graph = None  # Initialize the graph
        self.read_file()
        self.get_logger().info(f"BFS path from 0 to 3: {self.bfs(0, 9)}") 

    def read_file(self):
        try:
            with open("roadmap.geojson", "r") as file:
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


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()