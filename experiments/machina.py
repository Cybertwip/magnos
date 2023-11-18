import pygame
import sys

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Press and Iron Bars")

# Colors
blue = (0, 0, 255)
red = (255, 0, 0)
green = (0, 255, 0)

# Bar dimensions
bar_width = 20
bar_height = 150
bar_distance = 0

# Bar positions
press_left_x = width // 4 - bar_width // 2
press_right_x = width - width // 4 - bar_width // 2
iron_left_x = press_left_x - bar_distance - bar_width
iron_right_x = press_right_x + bar_width

# Bar speed
speed = 20

# Circle properties
circle_radius = 15
circle_x = 0  # Initial position at the leftmost side
circle_y = height // 2

# Circle speed
circle_speed = 30

# Flags for bar movement
press_opening = True  # Start with the press opening
iron_left_closing = True  # Start with the left iron bar closing
iron_right_closing = True  # Start with the right iron bar closing

# Timer for automatic press opening/closing
pygame.time.set_timer(pygame.USEREVENT, 500)  # Set the timer to toggle every 2000 milliseconds (2 seconds)

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.USEREVENT:
            # Toggle the press opening/closing automatically
            press_opening = not press_opening
            iron_left_closing = not iron_left_closing
            iron_right_closing = not iron_right_closing

    # Update bar positions based on flags
    if press_opening and press_left_x < width // 2 - bar_width // 2:
        press_left_x += speed
        press_right_x -= speed
    elif not press_opening and press_left_x > width // 4 - bar_width // 2:
        press_left_x -= speed
        press_right_x += speed

    # Update circle position
    if circle_x < width - circle_radius:
        circle_x += circle_speed
    else:
        circle_x = 0

    # Draw the bars
    screen.fill((255, 255, 255))  # Fill the screen with white

    # Draw the circle
    pygame.draw.circle(screen, green, (int(circle_x), int(circle_y)), circle_radius)

    pygame.draw.rect(screen, blue, (press_left_x, height // 2 - bar_height // 2, bar_width, bar_height))  # Left blue press bar
    pygame.draw.rect(screen, blue, (press_right_x, height // 2 - bar_height // 2, bar_width, bar_height))  # Right blue press bar
    pygame.draw.rect(screen, red, (iron_left_x, height // 2 - bar_height // 2, bar_width, bar_height))  # Left red iron bar
    pygame.draw.rect(screen, red, (iron_right_x, height // 2 - bar_height // 2, bar_width, bar_height))  # Right red iron bar


    pygame.display.flip()  # Update the display
    pygame.time.Clock().tick(120)  # Cap the frame rate to 30 frames per second
