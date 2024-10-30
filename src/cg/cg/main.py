import pygame
from .Utils.Csv import load_from_csv
from .Maze import Maze


def main():
    pygame.init()
    maze = Maze(load_from_csv("maps/default.csv"))
    maze.run()
