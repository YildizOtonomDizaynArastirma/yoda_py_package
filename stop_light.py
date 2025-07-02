#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from transformations import quaternion_from_euler

class SpawnModelNode(Node):
    def __init__(self):
        super().__init__('spawn_model_node')
        
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        while not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service...')

        self.spawned_models = []  # Keep track of spawned entities

    def delete_previous(self):
        """Deletes all previously spawned entities before spawning new ones."""
        for model in self.spawned_models:
            request = DeleteEntity.Request()
            request.name = model
            future = self.delete_cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Deleted entity: {model}')
            else:
                self.get_logger().error(f'Failed to delete entity: {model}')
        
        self.spawned_models = []  # Clear the list

    def spawn_sdf(self, model_name_prefix, sdf_file_path, pos_list):
        self.delete_previous()  # Ensure all previous models are removed

        for i, (x, y, z, roll, pitch, yaw) in enumerate(pos_list):
            unique_model_name = f"{model_name_prefix}_{i}"

            with open(sdf_file_path, 'r') as sdf_file:
                sdf_xml = sdf_file.read()

            quat = quaternion_from_euler(roll, pitch, yaw)

            request = SpawnEntity.Request()
            request.name = unique_model_name
            request.xml = sdf_xml
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = z
            request.initial_pose.orientation.x = quat[0]
            request.initial_pose.orientation.y = quat[1]
            request.initial_pose.orientation.z = quat[2]
            request.initial_pose.orientation.w = quat[3]

            future = self.spawn_cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Spawned [{unique_model_name}]: {future.result().status_message}')
                self.spawned_models.append(unique_model_name)  # Store spawned model name
            else:
                self.get_logger().error(f'Failed to spawn entity: {unique_model_name}')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnModelNode()

    base_path = '/home/safa/yoda_ws/src/yoda_cpp_package/meshes'
    lights = {
        'g': {
            'name_prefix': 'traffic_light_green',
            'sdf_path': f'{base_path}/green.sdf',
            'pos_list': [
                (-38.599534, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-36.693382, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (-12.178672, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-9.857101, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (68.473506, -2.442062, 1.775000, -1.5708, 0.0, 0.0),
                (66.241683, -2.442062, 3.457963, -1.5708, 0.0, 0.0)
            ]
        },
        'r': {
            'name_prefix': 'traffic_light_red',
            'sdf_path': f'{base_path}/red.sdf',
            'pos_list': [
                (-38.599534, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-36.693382, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (-12.178672, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-9.857101, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (68.473506, -2.442062, 1.775000, -1.5708, 0.0, 0.0),
                (66.241683, -2.442062, 3.457963, -1.5708, 0.0, 0.0)
            ]
        },
        'y': {
            'name_prefix': 'traffic_light_yellow',
            'sdf_path': f'{base_path}/yellow.sdf',
            'pos_list': [
                (-38.599534, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-36.693382, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (-12.178672, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-9.857101, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (68.473506, -2.442062, 1.775000, -1.5708, 0.0, 0.0),
                (66.241683, -2.442062, 3.457963, -1.5708, 0.0, 0.0)
            ]
        },
        'a': {
            'name_prefix': 'traffic_light_all',
            'sdf_path': f'{base_path}/all.sdf',
            'pos_list': [
                (-38.599534, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-36.693382, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (-12.178672, 9.238252, 1.775000, 1.5708, 0.0, 0.0),
                (-9.857101, 9.238252, 3.457963, 1.5708, 0.0, 0.0),
                (68.473506, -2.442062, 1.775000, -1.5708, 0.0, 0.0),
                (66.241683, -2.442062, 3.457963, -1.5708, 0.0, 0.0)
            ]
        }
    }

    while rclpy.ok():
        cmd = input("Enter [g]reen, [r]ed, [y]ellow, [a]ll, or [q]uit: ").strip().lower()
        if cmd == 'q':
            node.get_logger().info("Exiting.")
            break

        if cmd in lights:
            data = lights[cmd]
            node.spawn_sdf(data['name_prefix'], data['sdf_path'], data['pos_list'])
        else:
            node.get_logger().warn("Invalid choice.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

