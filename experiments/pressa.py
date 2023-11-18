import numpy as np
import pygame
from pygame.locals import QUIT, KEYDOWN, K_UP, K_DOWN
from qutip import *
import matplotlib.pyplot as plt

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
pygame.display.set_caption("Earth Atmosphere Simulation")

# Iron bars settings
iron_bar_width = 20
iron_bar_height = 150
iron_bar_color = (255, 0, 0)  # Red color

# Camera settings (spaceship)
camera_speed = 2
camera_y = height // 2  # Initial camera position

# Clock for controlling the frame rate
clock = pygame.time.Clock()

# Font settings
font = pygame.font.Font(None, 36)  # Choose your font and size
label_color = (0, 0, 0)  # Black color

# Earth atmospheric layers (using real Earth values)
troposphere_thickness = 8000  # meters
stratosphere_thickness = 32000  # meters
mesosphere_thickness = 50000  # meters
thermosphere_thickness = 200000  # meters

layer_colors = [
    (135, 206, 250),   # Sky Blue for troposphere
    (0, 191, 255),     # Deep Sky Blue for stratosphere
    (70, 130, 180),    # Steel Blue for mesosphere
    (25, 25, 112)      # Midnight Blue for thermosphere
]

layer_thicknesses = [troposphere_thickness, stratosphere_thickness, mesosphere_thickness, thermosphere_thickness]

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    H_field = a.dag() * a
    H_left_iron = args['left_component_mass'] * (a.dag() + a)
    H_right_iron = args['right_component_mass'] * (a.dag() + a)
    return H_field + H_left_iron + H_right_iron

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_component_mass': 1, 'right_component_mass': 1}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args))

# Main simulation loop
running = True
i = 0

update_label_interval = 1#int(num_time_steps / (total_time))  # Update label every pi seconds
label_update_countdown = update_label_interval

# Create left and right EMF labels
left_label_text = f'Left EMF: {0:.2f} m/s²'
right_label_text = f'Right EMF: {0:.2f} m/s²'
current_time_minus_pi_label_text = f'Altitude: {0:.2f}'

# Variables for adjusting pi interval
pi_increase = 0
min_pi_interval = 0
max_pi_interval = np.pi

# Gravity offset settings
gravity_offset = 0
gravity_acceleration = 0.01  # Adjust this value to control acceleration

g = 9.81

# Find the initial y position of the iron bars
right_bar_y = height // 2 - iron_bar_height // 2
left_bar_y = height // 2 - iron_bar_height // 2

# Variables for time counter and polarity swapping
half_pi_time_reached = False
polarity_1l = 1  # Initial polarity
polarity_1r = 1  # Initial polarity
polarity_factor_1l = -1
polarity_factor_1r = 1


polarity_2l = 1  # Initial polarity
polarity_2r = 1  # Initial polarity
polarity_factor_2l = -1
polarity_factor_2r = 1

polarity_3l = 1  # Initial polarity
polarity_3r = 1  # Initial polarity
polarity_factor_3l = -1
polarity_factor_3r = 1

time_counter_1 = 0
time_counter_2 = 0
time_counter_3 = 0


# Calculate EMF values
emf_left_values = np.real(result.expect[0])
emf_right_values = np.real(-result.expect[1])

# Create separate plots for the left and right iron bars with values
plt.figure(1)
plt.plot(times, emf_left_values, label='Left Iron Bar (EMF)')
plt.plot(times, emf_right_values, label='Right Iron Bar (EMF)')
plt.xlabel('Time')
plt.ylabel('EMF')
plt.legend()

g = 9.81  # Acceleration due to gravity (m/s^2)

# Calculate EMF values in m/s²
emf_left_m_per_s_squared = g - (emf_left_values * g) 
emf_right_m_per_s_squared = g - (emf_right_values * g)

# Create separate plots for the left and right iron bars with values in m/s²
plt.figure(2)
plt.plot(times, emf_left_m_per_s_squared, label='Left Iron Bar (EMF in m/s^2)')
plt.plot(times, emf_right_m_per_s_squared, label='Right Iron Bar (EMF in m/s^2)')
plt.xlabel('Time')
plt.ylabel('EMF (m/s^2)')
plt.legend()

# Calculate the offsets
gravity_offset = emf_left_m_per_s_squared - emf_right_m_per_s_squared

# Plot the offsets for the left and right iron bars
plt.figure(3)
plt.plot(times, gravity_offset * np.ones(len(times)), label='Gravitational offset')
plt.xlabel('Time')
plt.ylabel('Offset (m/s^2)')
plt.legend()

plt.show()

# Earth parameters
earth_mass = 5.972e24  # kg
earth_radius = 6.371e6  # meters
# Gravitational constant
G = 6.67430e-11  # m^3 kg^-1 s^-2

max_altitude = 0
min_gravity = 10

gravity_turn = 0

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

    # Accelerate gravity offset
    if gravity_turn == 0:
        left_bar_y -= emf_left_m_per_s_squared * polarity_1l
        right_bar_y -= emf_right_m_per_s_squared * polarity_1r

        left_bar_osc = emf_left_m_per_s_squared * polarity_1l
        right_bar_osc = emf_left_m_per_s_squared * polarity_1l

    if gravity_turn == 1:
        left_bar_y -= emf_left_m_per_s_squared * polarity_2l
        right_bar_y -= emf_right_m_per_s_squared * polarity_2r

        left_bar_osc = emf_left_m_per_s_squared * polarity_2l
        right_bar_osc = emf_left_m_per_s_squared * polarity_2r

    if gravity_turn == 3:
        left_bar_y -= emf_left_m_per_s_squared * polarity_3l
        right_bar_y -= emf_right_m_per_s_squared * polarity_3l
        left_bar_osc = emf_left_m_per_s_squared * polarity_2l
        right_bar_osc = emf_left_m_per_s_squared * polarity_3r

    # Update camera position based on the average position of iron bars
    camera_y = (left_bar_y + right_bar_y) // 2
    camera_y -= iron_bar_height * 1.5

    left_bar_osc += camera_y + iron_bar_height * 1.5
    right_bar_osc += camera_y + iron_bar_height * 1.5

    # Clear the screen
    screen.fill((255, 255, 255))  # White background

    # Draw Earth atmospheric layers
    drawing_y = camera_y
    for thickness, color in zip(layer_thicknesses, layer_colors):
        pygame.draw.rect(screen, color, (0, height - thickness - drawing_y, width, thickness))
        drawing_y += thickness

    # Check if it's time to swap polarity
    if time_counter_1 >= total_time / 8 and time_counter_1 <= (total_time / 8) * 3:
        iron_bar_color = (0, 255, 0)  # Change color to green
    else:
        iron_bar_color = (255, 0, 0)  # Change color to red

    if time_counter_1 >= total_time / 2 and gravity_turn == 0:
        time_counter_1 = 0
        time_counter_2 = 0
        time_counter_3 = 0
        polarity_factor_1l *= -1
        polarity_factor_1r *= -1

        polarity_1l = polarity_factor_1l # Swap polarity
        polarity_1r = polarity_factor_1r # Swap polarity

        if gravity_turn == 0:
            gravity_turn = 1

    if time_counter_2 >= total_time / 8 and time_counter_2 <= (total_time / 8) * 3 and gravity_turn == 3:
        time_counter_1 = 0
        time_counter_2 = 0
        time_counter_3 = 0

        polarity_2l = polarity_factor_2l # Swap polarity
        polarity_2r = polarity_factor_2r # Swap polarity

        polarity_factor_2l *= -1
        polarity_factor_2r *= -1

        if gravity_turn == 1:
            gravity_turn = 2

    if time_counter_3 >= total_time / 8 and time_counter_3 <= (total_time / 8) * 3 and gravity_turn == 3:
        time_counter_1 = 0
        time_counter_2 = 0
        time_counter_3 = 0
        polarity_factor_3l *= -1
        polarity_factor_3r *= -1

        polarity_3l = polarity_factor_3l # Swap polarity
        polarity_3r = polarity_factor_3l # Swap polarity

        if gravity_turn == 2:
            gravity_turn = 0


    # Calculate altitude above Earth's surface
    altitude = -camera_y  # Assuming camera_y is the altitude above the surface
    if altitude < 0:
        altitude = 0  # Prevent altitude from going below 0

    if altitude > max_altitude:
        max_altitude = altitude

    # Calculate gravitational acceleration
    gravity = G * earth_mass / (earth_radius + altitude)**2

    # Update the acceleration due to gravity in your code
    g = gravity

    if g < min_gravity:
        min_gravity = g

    # Draw iron bars with inverted polarity after reaching half pi time
    pygame.draw.rect(screen, iron_bar_color, (width // 4 - iron_bar_width // 2, left_bar_osc - camera_y, iron_bar_width, iron_bar_height))
    pygame.draw.rect(screen, iron_bar_color, (3 * width // 4 - iron_bar_width // 2, right_bar_osc - camera_y, iron_bar_width, iron_bar_height))

    # Draw the ground
    pygame.draw.line(screen, (0, 0, 0), (0, height - camera_y), (width, height - camera_y), 2)

    # Update label every pi seconds
    label_update_countdown -= 1
    if label_update_countdown == 0:
        label_update_countdown = update_label_interval

        # Create left and right EMF labels
        left_label_text = f'Left EMF: {emf_left_m_per_s_squared:.2f} m/s²'
        right_label_text = f'Right EMF: {emf_right_m_per_s_squared:.2f} m/s²'

        # Calculate current time - PI
        current_time_minus_pi = pi_increase
        current_time_minus_pi_label_text = f'Altitude: {max_altitude:.2f}'

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

    # Increment time step and time counter
    i += 1
    if gravity_turn == 0:
        time_counter_1 += (total_time) * 1.0 / 24.0

    if gravity_turn == 1:
        time_counter_2 += (total_time) * 1.0 / 24.0

    if gravity_turn == 2:
        time_counter_3 += (total_time) * 1.0 / 24.0

    if i >= num_time_steps:
        i = 0

pygame.quit()
