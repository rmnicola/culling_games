import pygame
import sys
import threading

import rclpy
from cg_interfaces.srv import MoveCmd, GetMap, Reset
from cg_interfaces.msg import RobotSensors
from rclpy.node import Node

from .Maze import Maze
from .Robot import Robot
from .Utils.Finders import find
from .Utils.Grid import flatten
from .Utils.Csv import load_from_csv


class Game(Node):
    def __init__(self, map_path, resolution=720):
        super().__init__("Culling_Games")
        self.map_path = map_path
        self.resolution = resolution
        
        self.move_cmd_service = self.create_service(MoveCmd, 'move_command', self.handle_move_cmd)
        self.get_map_service = self.create_service(GetMap, 'get_map', self.handle_map_request)
        self.reset_service = self.create_service(Reset, 'reset', self.handle_reset_request)
        
        self.publisher_ = self.create_publisher(RobotSensors, '/culling_games/robot_sensors', 10)
        self.timer = self.create_timer(0.5, self.publish_sensor_data)
        
        initial_maze_config = load_from_csv(self.map_path)
        self.maze = Maze(initial_maze_config, self.resolution)
        self.robot = Robot(self.maze)
        
        self.running = False
        self.win = False
        self.win_handled = False
        
        self.reset_requested = False
        self.state_lock = threading.Lock()

    def handle_reset_request(self, request, response):
        with self.state_lock:
            self.get_logger().info('Reset request received.')
            self.reset_requested = True
        response.success = True
        return response

    def publish_sensor_data(self):
        msg = RobotSensors()
        surroundings = self.robot.check_surroundings()
        msg.up = surroundings['up']
        msg.down = surroundings['down']
        msg.left = surroundings['left']
        msg.right = surroundings['right']
        msg.up_left = surroundings['up_left']
        msg.up_right = surroundings['up_right']
        msg.down_left = surroundings['down_left']
        msg.down_right = surroundings['down_right']
        self.publisher_.publish(msg)

    def update(self):
        with self.state_lock:
            if self.reset_requested:
                self.get_logger().info('Resetting the game board from file...')
                new_maze_config = load_from_csv(self.map_path)
                self.maze = Maze(new_maze_config, self.resolution)
                self.robot = Robot(self.maze)
                self.win = False
                self.win_handled = False
                self.reset_requested = False

        self.handle_input()
        if not self.win:
            self.maze.draw()

    def handle_map_request(self, request, response):
        occ_grid = self.maze.get_occupancy_grid()
        response.occupancy_grid_flattened = flatten(occ_grid)
        response.occupancy_grid_shape = len(occ_grid), len(occ_grid[0])
        return response

    def handle_move_cmd(self, request, response):
        direction = request.direction.lower()
        target_pos = find(self.maze.get_occupancy_grid(), 't')
        response.success = self.robot.move(direction)
        if target_pos == self.robot.pos:
            self.win = True
        response.robot_pos = self.robot.pos
        response.target_pos = target_pos
        return response

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_h:
                    direction = 'left'
                elif event.key == pygame.K_j:
                    direction = 'down'
                elif event.key == pygame.K_k:
                    direction = 'up'
                elif event.key == pygame.K_l:
                    direction = 'right'
                else:
                    return
                target_pos = find(self.maze.get_occupancy_grid(), 't')
                success = self.robot.move(direction)
                if target_pos == self.robot.pos:
                    self.win = True
                    self.maze.win()
        
    def run(self):
        pygame.init()
        self.running = True
        while self.running and rclpy.ok():
            self.update()
            pygame.display.flip()

            if self.win and not self.win_handled:
                self.win_handled = True
                self.maze.win()

        pygame.quit()
