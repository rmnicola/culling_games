import pygame

from .Utils.Screen import generate_screen

OFFWHITE = (220, 224, 232)
BLACK = (0, 0, 0)
GREY = (35, 38, 52)
RED = (230, 69, 83)
BLUE = (114, 135, 253)


class Maze:
    def __init__(self, maze_initial_configuration, resolution):
        self.occupancy_grid = maze_initial_configuration
        self.screen, self.grid_size = generate_screen(self.occupancy_grid.copy(),
                                                      resolution)
        self.screen_width, self.screen_height = self.screen.get_size()

    def draw(self):
        self.screen.fill(OFFWHITE)  # Black background
        # Draw the maze
        for i, row in enumerate(self.occupancy_grid):
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

    def set_grid(self, pos, value):
        x, y = pos
        self.occupancy_grid[x][y] = value
