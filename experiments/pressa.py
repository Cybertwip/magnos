import numpy as np
import pygame
from pygame.locals import QUIT, KEYDOWN, K_UP, K_DOWN
from qutip import *

pygame.init()

# Constants
N = 2
n_photons = 0
a = destroy(N)
H_field = a.dag() * a
psi0 = fock(N, n_photons)
total_time = 3.14
num_time_steps = 200
times = np.linspace(0, total_time, num_time_steps)

# Pygame settings
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Iron Bars Simulation")

# Iron bars settings
iron_bar_width = 20
iron_bar_height = 150
iron_bar_color = (255, 0, 0)  # Red color

# Clock for controlling the frame rate
clock = pygame.time.Clock()

# Font settings
font = pygame.font.Font(None, 36)  # Choose your font and size
label_color = (0, 0, 0)  # Black color

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    H_field = a.dag() * a
    H_left_iron = args['left_component_mass'] * (a.dag() + a)
    H_right_iron = args['right_component_mass'] * (a.dag() + a)
    return H_field + H_left_iron + H_right_iron

# Initialize lists to store EMF values for left and right bars
emf_left_values = []
emf_right_values = []

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_component_mass': 1.0, 'right_component_mass': 1.0}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args))

# Main simulation loop
running = True
i = 0

update_label_interval = int(num_time_steps / (total_time))  # Update label every pi seconds
label_update_countdown = update_label_interval

# Create left and right EMF labels
left_label_text = f'Left EMF: {0:.2f} m/s²'
right_label_text = f'Right EMF: {0:.2f} m/s²'
current_time_minus_pi_label_text = f'PI time measure: {0:.2f}'

# Variables for adjusting pi interval
pi_increase = 0
min_pi_interval = 0
max_pi_interval = np.pi

g = 9.81 

while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN:
            if event.key == K_UP:
                pi_increase += np.pi * 0.25  # Increase the interval
                pi_increase = min(pi_increase, max_pi_interval)

            elif event.key == K_DOWN:
                pi_increase -= np.pi * 0.25  # Decrease the interval
                pi_increase = max(pi_increase, min_pi_interval)

    # Calculate EMF values in m/s² for the current time step
    emf_left_m_per_s_squared = g - (result.expect[0][i] * g)
    emf_right_m_per_s_squared = g - (-result.expect[1][i] * g)

    # Calculate the offsets
    gravity_offset = emf_left_m_per_s_squared - emf_right_m_per_s_squared

    # Update iron bars based on EMF values
    left_bar_y = height // 2 - iron_bar_height // 2 + gravity_offset
    right_bar_y = height // 2 - iron_bar_height // 2 - gravity_offset

    # Clear the screen
    screen.fill((255, 255, 255))  # White background

    # Draw iron bars
    pygame.draw.rect(screen, iron_bar_color, (width // 4 - iron_bar_width // 2, left_bar_y, iron_bar_width, iron_bar_height))
    pygame.draw.rect(screen, iron_bar_color, (3 * width // 4 - iron_bar_width // 2, right_bar_y, iron_bar_width, iron_bar_height))

    # Find the minimum y position of the iron bars
    min_y_position = max(left_bar_y, right_bar_y)

    # Draw the ground
    pygame.draw.line(screen, (0, 0, 0), (0, min_y_position + iron_bar_height), (width, min_y_position + iron_bar_height), 2)

    # Update label every pi seconds
    label_update_countdown -= 1
    if label_update_countdown == 0:
        label_update_countdown = update_label_interval

        # Create left and right EMF labels
        left_label_text = f'Left EMF: {emf_left_m_per_s_squared:.2f} m/s²'
        right_label_text = f'Right EMF: {emf_right_m_per_s_squared:.2f} m/s²'

        # Calculate current time - PI
        current_time_minus_pi = pi_increase
        current_time_minus_pi_label_text = f'PI time measure: {current_time_minus_pi:.2f}'

    # Render labels
    left_label_render = font.render(left_label_text, True, label_color)
    right_label_render = font.render(right_label_text, True, label_color)
    current_time_minus_pi_label_render = font.render(current_time_minus_pi_label_text, True, label_color)

    # Draw labels on the screen
    screen.blit(left_label_render, (10, 10))
    screen.blit(right_label_render, (width - right_label_render.get_width() - 10, 10))
    screen.blit(current_time_minus_pi_label_render, (width // 2 - current_time_minus_pi_label_render.get_width() // 2, 10))

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    clock.tick(120)

    # Increment time step
    i += 1
    if i >= num_time_steps:
        i = 0

pygame.quit()




