import pygame
import sys
from collections import namedtuple

Res = namedtuple('Resolution', ['W', 'H'])

OFFWHITE = (255, 255, 250)
BLACK = (0, 0, 0)
RESOLUTION = Res(W=1280, H=720)


class Maze:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.width = RESOLUTION.W
        self.height = RESOLUTION.H
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Simple Maze Grid")
        self.screen.fill(OFFWHITE)  # Black background

    def draw_grid(self):
        # Draw vertical lines
        for x in range(0, self.width, self.grid_size):
            pygame.draw.line(self.screen, BLACK, (x, 0), (x, self.height))
        
        # Draw horizontal lines
        for y in range(0, self.height, self.grid_size):
            pygame.draw.line(self.screen, BLACK, (0, y), (self.width, y))

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            # Draw the grid each frame
            self.draw_grid()
            pygame.display.flip()

        pygame.quit()
        sys.exit()
