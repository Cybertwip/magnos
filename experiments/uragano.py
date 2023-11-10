import pygame
import sys
import random
import time
from pygame.locals import *
import math

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
HURRICANE_SIZE = 100
BOMB_RADIUS = 30
BOMB_SPEED = 1  # Speed for moving the bomb
RESET_DELAY = 1  # 1 second delay before resetting the hurricane
APERTURE = 2000  # Aperture for the curved path
ACCELERATION = 0.00001  # Exponential acceleration factor

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Initialize the screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Hurricane and Oxygen Bomb Simulator")

# Initialize hurricane
hurricane_x = 0
hurricane_y = HEIGHT
hurricane_velocity = [1, -1]  # Initial velocity

# Initialize oxygen bomb
bomb_x = random.randint(0, WIDTH)
bomb_y = random.randint(0, HEIGHT)

# Flags to indicate if the hurricane should reset and delay the reset
reset_hurricane = False
reset_start_time = 0

# Set up the clock for controlling the frame rate
clock = pygame.time.Clock()
FPS = 60

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    # Move the bomb with arrow keys
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        bomb_x -= BOMB_SPEED
    if keys[K_RIGHT]:
        bomb_x += BOMB_SPEED
    if keys[K_UP]:
        bomb_y -= BOMB_SPEED
    if keys[K_DOWN]:
        bomb_y += BOMB_SPEED

    if reset_hurricane:
        # Check if the delay has passed
        current_time = time.time()
        if current_time - reset_start_time >= RESET_DELAY:
            # Reset the hurricane to a starting position
            hurricane_x = 0
            hurricane_y = HEIGHT
            hurricane_velocity = [1, -1]  # Reset velocity
            reset_hurricane = False
    else:
        # Update the hurricane's position using exponential acceleration
        hurricane_x += hurricane_velocity[0]
        hurricane_y += hurricane_velocity[1]
        hurricane_velocity[0] += ACCELERATION * (hurricane_x + 1)
        hurricane_velocity[1] += ACCELERATION * (HEIGHT - hurricane_y)

        # Check if the bomb is within the hurricane
        distance = ((hurricane_x - bomb_x) ** 2 + (hurricane_y - bomb_y) ** 2) ** 0.5
        if distance < HURRICANE_SIZE / 2 and keys[K_SPACE]:
            # Start the reset delay
            reset_start_time = time.time()
            reset_hurricane = True

    # Ensure the hurricane and bomb stay within the screen bounds
    hurricane_x = max(0, min(WIDTH - HURRICANE_SIZE, hurricane_x))
    hurricane_y = max(0, min(HEIGHT - HURRICANE_SIZE, hurricane_y))
    bomb_x = max(0, min(WIDTH, bomb_x))
    bomb_y = max(0, min(HEIGHT, bomb_y))

    # Fill the screen
    screen.fill(WHITE)

    if not reset_hurricane:
        # Draw the hurricane
        pygame.draw.circle(screen, BLUE, (int(hurricane_x), int(hurricane_y)), HURRICANE_SIZE // 2)

    # Draw the oxygen bomb
    pygame.draw.circle(screen, RED, (bomb_x, bomb_y), BOMB_RADIUS)

    # Update the display
    pygame.display.update()

    # Control the frame rate
    clock.tick(FPS)

# Quit Pygame
pygame.quit()
sys.exit()
