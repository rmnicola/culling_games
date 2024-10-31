import pygame
import sys

from .Maze import Maze
from .Robot import Robot
from .Utils.Finders import find


class Game:
    def __init__(self, maze_initial_configuration, resolution=720):
        self.maze = Maze(maze_initial_configuration, resolution)
        self.running = False
        self.robot = Robot(self.maze)

    def update(self):
        self.handle_input()
        self.maze.draw()

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
                success, surroundings = self.robot.move(direction)
                if success:
                    print(f"Robo movido! Robo vê: {surroundings}")
                else:
                    print(f"O movimento falhou! Robo vê: {surroundings}")
        
    def run(self):
        pygame.init()
        self.running = True
        while self.running:
            self.update()
            pygame.display.flip()
        pygame.quit()
        sys.exit()
