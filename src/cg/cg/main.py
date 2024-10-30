from .Utils.Csv import load_from_csv
from .Maze import Maze
from .Game import Game


def main():
    game = Game(load_from_csv("maps/default.csv"))
    game.run()
