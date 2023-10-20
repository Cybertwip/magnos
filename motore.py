import pygame
import sys
import math

# Pygame setup
pygame.init()

# Constants
WIDTH, HEIGHT = 400, 400
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GRAY = (128, 128, 128)

# Initialize screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Wheel Motor Simulation")

# Initial angles and angular velocity
outer_plate_angle = 0
inner_plate_angle = 0
angular_velocity = 0  # Start with 0 angular velocity
wheel_angular_velocity = 0

# Time and RPM calculations
time_elapsed = 0
max_rpm = 5000  # Max RPM for 12 volts (for illustration purposes)

coil_inductance = 10.0  # Example coil inductance (you can adjust this)
current = 120  # Example current input (you can adjust this)

clock = pygame.time.Clock()

spinning = False  # Flag to indicate if the inner plate is spinning

running = True

wheel_angular_velocity = 0

wheel_angle = 0

# Draw the components
wheel_radius = 100
brake_plate_radius = 80
motion_plate_radius = 40


wheel_center = (WIDTH // 2, HEIGHT // 2)
inner_plate_center = wheel_center


# Calculate electromagnetic force (simplified) based on coil inductance and current
electromagnetic_force = coil_inductance * current

# Precompute torque and angular acceleration
torque = electromagnetic_force * motion_plate_radius
angular_acceleration = torque / (0.5 * wheel_radius ** 2)  # Moment of inertia for a circular plate


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                spinning = not spinning  # Toggle spinning on/off
                angular_velocity = 360 if spinning else 0

    screen.fill(WHITE)


    # Draw the wheel
    pygame.draw.circle(screen, GRAY, wheel_center, wheel_radius)
    # Draw the brake plate
    pygame.draw.circle(screen, RED, inner_plate_center, brake_plate_radius)
    # Draw the motion plate
    pygame.draw.circle(screen, GRAY, inner_plate_center, motion_plate_radius)

    # Rotate the inner plate if spinning
    if spinning:
        inner_plate_angle += angular_velocity * clock.get_time() / 1000
        inner_plate_angle %= 360

    if spinning:
        wheel_angular_velocity += angular_acceleration * clock.get_time() / 1000
        wheel_angle += wheel_angular_velocity * clock.get_time() / 1000

    # Draw lines pointing north, south, east, and west on the inner and middle plates
    line_length = motion_plate_radius
    for angle in range(0, 360, 90):
        angle_rad = math.radians(inner_plate_angle + angle)  # Rotate the line with inner_plate_angle
        x1 = inner_plate_center[0] + math.cos(angle_rad)
        y1 = inner_plate_center[1] + math.sin(angle_rad)
        x2 = x1 + line_length * math.cos(angle_rad)
        y2 = y1 + line_length * math.sin(angle_rad)
        pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), 2)

    # line_length = brake_plate_radius
    # for angle in range(0, 360, 90):
    #     x1 = inner_plate_center[0] + (brake_plate_radius - motion_plate_radius) * math.cos(math.radians(angle))
    #     y1 = inner_plate_center[1] + (brake_plate_radius - motion_plate_radius) * math.sin(math.radians(angle))
    #     x2 = x1 + line_length * math.cos(math.radians(angle))
    #     y2 = y1 + line_length * math.sin(math.radians(angle))
    #     #pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), 2)

    line_length = brake_plate_radius
    angle_rad = math.radians(wheel_angle)  # Rotate the line with inner_plate_angle
    x1 = inner_plate_center[0] + (wheel_radius - motion_plate_radius + motion_plate_radius * 0.5) * math.cos(angle_rad)
    y1 = inner_plate_center[1] + (wheel_radius - motion_plate_radius + motion_plate_radius * 0.5) * math.sin(angle_rad)
    x2 = x1 + line_length * math.cos(angle_rad)
    y2 = y1 + line_length * math.sin(angle_rad)
    pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), 2)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
