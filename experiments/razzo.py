from qiskit import Aer, QuantumCircuit, transpile, execute
import qutip as qt
import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame.locals import *

MAX_QUBITS = 1024  # This should be a number high enough to accommodate all the qubits you expect to need

plate_effects = [0] * MAX_QUBITS
teleport_count = 0  # Initialize the teleportation count

def prepare_superposition_qubits(energy, existing_circuit):
    num_new_qubits = int(energy * 10)  # Produce a number of qubits proportional to the energy
    
    # Starting qubit index to apply superposition. Assuming you're adding qubits sequentially.
    start_idx = existing_circuit.num_qubits - MAX_QUBITS
    
    # Put these new qubits into a superposition state
    for q in range(start_idx, start_idx + num_new_qubits):
        existing_circuit.h(q)

    return existing_circuit

def excite_iron_spin(circuit):
    # Assuming the second qubit represents the spin state of Iron
    theta = 3.14  # Hypothetical angle representing the magnetic field's effect on Iron's spin
    circuit.rz(theta, 1)  # Apply a magnetic field using Rz gate
    circuit.x(1)  # Spin-flip using a microwave pulse akin to Pauli-X gate
    return circuit


def gravitational_potential_due_to_vacuum_plates(position: float = 0.5):
    """Calculate gravitational potential based on vacuum plate effects."""
    global plate_effects
    
    gravitational_potential = 9.8  # Earth's gravitational potential

    gravitational_shift = (position - 0.5) * 0.01  # Slight change here: Added * 0.1 to reduce the effect.
    gravitational_shift += sum(plate_effects) * 0.001  # Adjust due to the effects of vacuum plates, reduced the factor.
    gravitational_potential += gravitational_shift * 0.2

    return gravitational_potential



def draw_triangle(surface, color, center_position, size):
    """Draws a triangle given a center position (centroid) and size."""
    # Calculate top center point based on centroid
    top_center = (center_position[0], center_position[1] - size // 3)
    bottom_left = (top_center[0] - size // 2, top_center[1] + size)
    bottom_right = (top_center[0] + size // 2, top_center[1] + size)
    pygame.draw.polygon(surface, color, [top_center, bottom_left, bottom_right])

def start_pygame_emergence_simulation(gravitational_potential):
    pygame.init()
    pygame.font.init()  # Initialize the font module

    font = pygame.font.SysFont(None, 25)  # Choose any font and size you like

    #for i in range(0, 10):
        #teleport_vacuum_plates(False)


    global teleport_count

    teleport_count = 0
    teleport_count += 1

    plate_effects = [0.0001] * 1  # Assume 10 vacuum plates each adding 0.1 to the effect

    WINDOW_WIDTH = 1280
    WINDOW_HEIGHT = 900
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('Rocket Emergence Simulation')

    WHITE = (255, 255, 255)
    BLUE = (0, 0, 255)
    BLACK = (0, 0, 0)

    clock = pygame.time.Clock()
    running = True

    rocket_size = 50
    position_y = WINDOW_HEIGHT // 2 - 130  # Position it on the upper edge of the troposphere circle
    position_x = WINDOW_WIDTH // 2   # Center the rocket's x position
    move_speed = 5
    emerged = False
    auto_move_speed = 0

    button_color = (50, 200, 50)  # Green
    button_hover_color = (100, 250, 100)  # Lighter Green
    launch = False  # A flag to check if the button has been clicked
    
    launch = False  # A flag to check if the button has been clicked


    # Gravitational acceleration constant (in pixels per frame^2 for the sake of this simulation)
    GRAVITY = 0.2

    # Initial vertical velocity of the rocket
    vertical_velocity = 0

    # New Colors for the layers of the atmosphere
    troposphere_color = (0, 119, 190)   # Blue
    stratosphere_color = (135, 206, 250)  # Lighter blue
    mesosphere_color = (106, 90, 205)   # Purple
    thermosphere_color = (255, 69, 0)   # Reddish-orange
    exosphere_color = (25, 25, 112)     # Dark blue
    button_color = (50, 200, 50)        # Green for the button
    button_hover_color = (100, 250, 100)  # Lighter Green for the button when hovered

    button_rect = pygame.Rect((WINDOW_WIDTH - 100) // 2, WINDOW_HEIGHT - 50, 100, 40)

    def gravitational_potential_to_speed(gravitational_potential, position_y):
        base_speed_range = 1.0
        potential_range = 9.8 - 3.5
        
        # Calculate local gravitational potential based on rocket's altitude
        local_potential = 9.8 * (position_y / (WINDOW_HEIGHT // 2 - 75 - rocket_size // 2))
        adjusted_potential = gravitational_potential - local_potential
        
        speed = 5 - (adjusted_potential - 3.5) * base_speed_range / potential_range
        
        return speed

    orbiting = False  # Flag to determine if the rocket is in orbit
    angle = 0  # Start angle for orbiting

    while running:
        screen.fill(exosphere_color)  # Set background to exosphere color

        # Drawing the atmosphere layers as concentric circles
        pygame.draw.circle(screen, thermosphere_color, (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), 300)  # Thermosphere
        pygame.draw.circle(screen, mesosphere_color, (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), 250)  # Mesosphere
        pygame.draw.circle(screen, stratosphere_color, (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), 200)  # Stratosphere
        pygame.draw.circle(screen, troposphere_color, (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), 150)  # Troposphere

        # Drawing the launch circle
        pygame.draw.circle(screen, BLACK, (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), 100, 2)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == MOUSEBUTTONDOWN:
                if button_rect.collidepoint(event.pos):
                    launch = True

        # If launch button was clicked, adjust rocket's movements
        if launch:
            # Calculate the gravitational speed due to potential
            gravitational_speed = gravitational_potential_due_to_vacuum_plates(position_y)

            # If the rocket has exited the exosphere
            if not orbiting and position_y < 100 - 25:  # Assuming 100 is the limit for orbital height
                gravitational_speed = 0
                vertical_velocity = 0
                GRAVITY = 0
                orbiting = True
                start_orbiting = True


            # Adjust the rocket's vertical movement based on gravitational speed
            position_y -= gravitational_speed



            # Adjust vertical velocity based on gravitational acceleration
            vertical_velocity += GRAVITY

            # Adjust rocket's position based on vertical velocity
            position_y += vertical_velocity

        if orbiting:
            orbit_radius = 350  # Define how wide the orbit is
            pygame.draw.circle(screen, (255, 0, 0), (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), orbit_radius, 1)  # Red circle
            
            if start_orbiting:
                angle = np.arctan2(position_y - WINDOW_HEIGHT // 2, position_x - WINDOW_WIDTH // 2)
                start_orbiting = False

            position_x = WINDOW_WIDTH // 2 + orbit_radius * np.cos(angle) 
            position_y = WINDOW_HEIGHT // 2 + orbit_radius * np.sin(angle) 

            position_y -= 25
            
            angle += 0.01  # Adjust this value to change the speed of orbiting

        # Restrict rocket's horizontal movement within window bounds
        position_x = max(0, min(position_x, WINDOW_WIDTH - rocket_size))

        pygame.draw.line(screen, BLACK, (0, WINDOW_HEIGHT // 2), (WINDOW_WIDTH, WINDOW_HEIGHT // 2), 2)
        draw_triangle(screen, BLUE, (position_x, position_y), rocket_size)

        # Drawing the button
        mouse_pos = pygame.mouse.get_pos()
        if button_rect.collidepoint(mouse_pos):
            pygame.draw.rect(screen, button_hover_color, button_rect)
        else:
            pygame.draw.rect(screen, button_color, button_rect)

        text_surface = font.render('LAUNCH', True, BLACK)
        screen.blit(text_surface, (button_rect.x + 15, button_rect.y + 10))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()




# If the teleportation led to a change in gravitational potential
start_pygame_emergence_simulation(9.8)