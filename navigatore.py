import pygame
import sys

# Initialize pygame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BACKGROUND_COLOR = (0, 0, 0)
MAGSAIL_COLOR = (255, 255, 255)
SOLAR_WIND_COLOR = (255, 200, 0)

# Magsail parameters
MASS = 1000  # kg
SOLAR_WIND_FORCE = 50  # Increased force for visibility in simulation
acceleration = SOLAR_WIND_FORCE / MASS
velocity = 0
TIME_FACTOR = 10  # This will simulate 10 seconds passing per frame

# Create screen and clock
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Magsail Simulation')
clock = pygame.time.Clock()

# Magsail's position
position = SCREEN_WIDTH // 4

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Update the magsail's position based on velocity
    velocity += acceleration * TIME_FACTOR
    position += int(velocity)

    # If magsail goes off-screen, reset it
    if position > SCREEN_WIDTH:
        position = SCREEN_WIDTH // 4
        velocity = 0

    # Draw everything
    screen.fill(BACKGROUND_COLOR)
    pygame.draw.circle(screen, SOLAR_WIND_COLOR, (SCREEN_WIDTH//5, SCREEN_HEIGHT//2), 30)  # Solar wind representation
    pygame.draw.rect(screen, MAGSAIL_COLOR, (position, SCREEN_HEIGHT//2 - 20, 40, 40))  # Magsail

    pygame.display.flip()
    clock.tick(60)
