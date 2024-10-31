import pygame
import sys

from .Maze import Maze
from .Utils.Finders import find, is_obstructed


class Game:
    def __init__(self, maze_initial_configuration, resolution=720):
        self.maze = Maze(maze_initial_configuration, resolution)
        pygame.init()
        self.running = False

    def update(self):
        self.handle_input()
        self.maze.draw()

    def move_robot(self, direction):
        robot_pos = find(self.maze.occupancy_grid.copy(), 'r')
        new_row, new_col = robot_pos
        if direction == 'left':
            new_col -= 1
        if direction == 'right':
            new_col += 1
        if direction == 'down':
            new_row += 1
        if direction == 'up':
            new_row -= 1
        new_pos = new_row, new_col
        if not is_obstructed(self.maze.occupancy_grid.copy(), new_pos):
            self.maze.set_cell(robot_pos, 'f')
            self.maze.set_cell(new_pos, 'r')

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running =False
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
                self.move_robot(direction)
        
    def run(self):
        self.running = True
        while self.running:
            self.update()
            pygame.display.flip()
        pygame.quit()
        sys.exit()
