import pygame
import random
import os


def generate_screen(maze, resolution):
    grid_size = resolution // len(maze)
    screen = pygame.display.set_mode(size=(resolution, resolution))
    pygame.display.set_caption("Culling Games")
    return screen, grid_size

def load_images_from_directory(directory):
    images = []
    for filename in os.listdir(directory):
        if filename.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
            image_path = os.path.join(directory, filename)
            images.append(pygame.image.load(image_path))
    return images

# Display a random image centered on the screen
def display_random_image(screen, directory="lobotomy_kaisen"):
    image = random.choice(load_images_from_directory(directory))
    screen_width, screen_height = screen.get_size()
    image_rect = image.get_rect(center=(screen_width // 2, screen_height // 2))
    screen.fill((0, 0, 0))  # Fill the screen with a black background
    screen.blit(image, image_rect)
    pygame.display.flip()
