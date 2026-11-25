"""Solve the maze using the full map and movement services."""

import rclpy
from rclpy.node import Node

from cg_interfaces.srv import GetMap, MoveCmd

from .pathfinding import shortest_path


def call_service(node, client, request):
    """Send one service request and wait for its response."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is None:
        raise RuntimeError('Service request failed')
    return result


def main(args=None):
    """Read the map, plan with BFS, and execute every step."""
    rclpy.init(args=args)
    node = Node('autonomous_maze_solver')
    try:
        map_client = node.create_client(GetMap, '/get_map')
        move_client = node.create_client(MoveCmd, '/move_command')
        while not map_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for /get_map...')
        while not move_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for /move_command...')

        map_response = call_service(node, map_client, GetMap.Request())
        path = shortest_path(map_response.occupancy_grid_flattened,
                             map_response.occupancy_grid_shape)
        if path is None:
            node.get_logger().error('No path to the target')
            return

        node.get_logger().info('Path found: {} moves'.format(len(path)))
        for direction, expected_position in path:
            request = MoveCmd.Request()
            request.direction = direction
            response = call_service(node, move_client, request)
            if not response.success or tuple(response.robot_pos) != expected_position:
                node.get_logger().error('Move failed or robot position changed')
                return
        node.get_logger().info('Target reached in {} moves'.format(len(path)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
