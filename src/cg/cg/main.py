from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor


def game():
    game = Game(load_from_csv("maps/default.csv"))
    game.run()

def editor():
    editor = Editor(load_from_csv("maps/default.csv"))
    editor.run()
