import pygame
from .Maze import Maze


def main():
    pygame.init()
    maze = Maze(grid_size=40)
    maze.run()
