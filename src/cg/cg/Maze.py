import pygame
import sys

from .Utils.Screen import generate_screen

OFFWHITE = (255, 255, 250)
BLACK = (0, 0, 0)
GREY = (125, 125, 125)
RED = (255, 0, 0)
BLUE = (0, 0, 255)


class Maze:
    def __init__(self, maze_initial_configuration, resolution=720):
        self.maze = maze_initial_configuration
        self.screen, self.grid_size = generate_screen(self.maze.copy(),
                                                      resolution)
        self.screen_width, self.screen_height = self.screen.get_size()

    def draw(self):
        self.screen.fill(OFFWHITE)  # Black background
        # Draw the maze
        for i, row in enumerate(self.maze):
            for j, cell in enumerate(row):
                if cell == 'r':
                    color = BLUE
                if cell == 't':
                    color = RED
                if cell == 'b':
                    color = GREY
                if cell == 'f':
                    color = OFFWHITE
                pygame.draw.rect(self.screen, color,
                                 pygame.Rect(j * self.grid_size, i *
                                             self.grid_size, self.grid_size,
                                             self.grid_size))

        # Draw vertical lines
        for x in range(0, self.screen_width, self.grid_size):
            pygame.draw.line(self.screen, BLACK, (x, 0), (x,
                                                          self.screen_height))
        
        # Draw horizontal lines
        for y in range(0, self.screen_height, self.grid_size):
            pygame.draw.line(self.screen, BLACK, (0, y),
                             (self.screen_width, y))


    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            # Draw the grid each frame
            self.draw()
            pygame.display.flip()

        pygame.quit()
        sys.exit()
