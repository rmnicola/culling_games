import pygame


def generate_screen(maze, resolution):
    grid_size = resolution // len(maze)
    screen = pygame.display.set_mode(size=(resolution, resolution))
    pygame.display.set_caption("Culling Games")
    return screen, grid_size
