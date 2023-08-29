import poppy
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pygame
import math
import numpy as np
import time

pygame.init()

# Initialize PyBullet
p.connect(p.DIRECT)

p.setGravity(0, 0, -10)

# Define magnet positions (up, down, left, right, front, back)
magnet_positions = [
    ((0, 1, 0), (0, 0, 1)),
    ((0, -1, 0), (0, 0, -1)),
    ((-1, 0, 0), (1, 0, 0)),
    ((1, 0, 0), (-1, 0, 0)),
    ((0, 0, 1), (0, -1, 0)),
    ((0, 0, -1), (0, 1, 0)),
]

# Create the magnet objects as bullet spheres
magnet_bodies = []
for color, position in magnet_positions:
    magnet_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
    magnet_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=magnet_shape, basePosition=position)
    magnet_bodies.append(magnet_body)


# Create the spaceship cone as a bullet box shape
cone_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
cone_body = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=cone_shape, basePosition=(0, 0, 2))

# Create the alternator object as a bullet sphere
alternator_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
alternator_body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=alternator_shape, basePosition=(0, 0, 0))

# Create constraints to attach magnets to the alternator
constraints = []
for magnet_body in magnet_bodies:
    constraint = p.createConstraint(
        parentBodyUniqueId=alternator_body,
        parentLinkIndex=-1,
        childBodyUniqueId=magnet_body,
        childLinkIndex=-1,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    constraints.append(constraint)

# Magnets power level
magnet_power = 0.00001

# Frame counter
frame_counter = 0

mouse_click = False 

# Electrical conversion efficiency (adjust this value as needed)
efficiency = 0.1

# Initialize variables for power and energy calculation
prev_time = time.time()
total_energy = 0.0
total_torque = np.zeros(3)  # Initialize total_torque here

# Calculate torque based on the magnetic interaction and apply to alternator
def apply_magnetic_torque():
    total_torque = np.zeros(3)
    
    for i, magnet_body in enumerate(magnet_bodies):
        magnet_position, _ = p.getBasePositionAndOrientation(magnet_body)
        alternator_position, _ = p.getBasePositionAndOrientation(alternator_body)

        # Calculate the normalized force vector
        force_vector = np.array(magnet_position) - np.array(alternator_position)
        force_vector /= np.linalg.norm(force_vector)

        # Calculate the torque vector and accumulate the total torque
        torque = np.cross(force_vector, alternator_position)
        total_torque += magnet_power

    # Apply the total torque to the alternator
    p.applyExternalTorque(alternator_body, -1, total_torque, p.LINK_FRAME)

    return total_torque

laser_shot = False
increase_power = False

# Electrical constants
resistance = 1.0  # Ohms (adjust this value as needed)
inductance = 0.1  # Henrys (adjust this value as needed)

# Initialize variables for current and magnetic field change rate
current = 0.0
magnetic_field_change_rate = 0.0

initial_position = np.array([0, 0, 0])
previous_position = initial_position
total_distance = 0.0

def current_to_lumens(current):
    """
    Convert electrical current (in mA) to luminous flux (in lumens).
    This is a simplified representation and may not represent the real behavior of an OLED device.
    """
    max_lumens = 300.0  # Maximum luminous flux (for a typical OLED, but this can vary)
    max_current = 20.0  # Maximum current input (this should be set based on OLED's real max current)
    
    lumens = (current / max_current) * max_lumens
    return np.clip(lumens, 0, max_lumens)


def simulate_oled_emission(current_input):
    lumens = current_to_lumens(current_input)
    
    # Assuming luminous flux linearly affects the aperture radius for simplicity
    aperture_radius = 0.5 * (lumens / 300.0)  # Arbitrary scaling factor
    
    # Create a circular aperture to represent the OLED emission area
    aperture = poppy.CircularAperture(radius=aperture_radius)
    
    # Create an optical system and add the OLED aperture
    osys = poppy.OpticalSystem()
    osys.add_pupil(aperture)
    osys.add_detector(pixelscale=0.1, fov_arcsec=5.0)
    
    # Create a monochromatic wavefront at 550 nm (green light for simplicity)
    green_wf = osys.input_wavefront(wavelength=550e-9)
    
    # Propagate the wavefront and compute the PSF
    psf = osys.calc_psf(wavelength=550e-9, display_intermediates=True)
    
    # Display the PSF
    print("OLED Emission PSF (Luminous Flux: {} Lumens)".format(lumens))


# Set up Pygame window
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption('OLED Brightness Simulation')


def energy_to_color(energy):
    """
    Convert accumulated energy (or any other value) to a grayscale color.
    Here we assume a simple linear relationship.
    """
    max_energy = 100.0  # Arbitrary max energy value for full brightness
    brightness = int((energy / max_energy) * 255)
    return np.clip(brightness, 0, 255)

def current_to_brightness(current):
    """
    Convert electrical current (in mA) to brightness (in arbitrary units).
    Here we assume a simple linear relationship.
    """
    max_brightness = 255.0  # Maximum achievable brightness (for Pygame)
    max_current = 10.0  # Maximum current input without damage
    
    brightness = (current / max_current) * max_brightness
    return np.clip(brightness, 0, max_brightness)

# Main loop
while True:
    # Check for mouse click manually using a keyboard key (e.g., 'a')
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                laser_shot = True

        # Key release events
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a:
                laser_shot = False

    keys = pygame.key.get_pressed()

    if keys[pygame.K_q]:
        increase_power = True
    else:
        increase_power = False

    # Apply the total torque to the alternator and get the total torque value
    if increase_power:
        total_torque = apply_magnetic_torque()

    # Calculate time difference since last frame
    current_time = time.time()
    time_difference = current_time - prev_time
    prev_time = current_time

    # Calculate rotational speed in radians per second
    angular_velocity = 2 * math.pi / time_difference

    # Calculate mechanical power
    mechanical_power = np.dot(total_torque, angular_velocity)

    # Calculate electrical power
    electrical_power = mechanical_power * efficiency

    # Calculate the energy generated by the alternator
    generated_energy = electrical_power * time_difference * efficiency

    # Accumulate the generated energy
    total_energy += generated_energy

    # Increase the magnet power using the accumulated energy
    magnet_power += generated_energy

    if increase_power:
        energy_used = electrical_power * time_difference
    else:
        energy_used = 0.0

    # Calculate the required input power to charge the magnets
    required_input_power = energy_used / time_difference

    # Print the estimated electrical power, accumulated energy, and required input power
    
    # Apply the total torque to the alternator
    if increase_power:
        apply_magnetic_torque()
        print("Estimated Electrical Power: {} W".format(electrical_power))
        print("Accumulated Energy: {} J".format(total_energy))
        print("Required Input Power to Charge Magnets: {} W".format(required_input_power))
            
    # Release space
    if laser_shot:
        #p.setGravity(0, 0, 0)
        
        # Calculate the change rate of magnetic field for the next frame
        magnetic_field_change_rate = (magnet_power - current) / inductance
        
        # Calculate the induced electromotive force (EMF) due to changing magnetic field
        induced_emf = -inductance * magnetic_field_change_rate

        # Calculate the induced current using Ohm's law (V = I * R)
        induced_current = induced_emf / resistance

        # Update the current value
        current = magnet_power

        print("Estimated Electrical Power: {} W".format(electrical_power))
        print("Current: {} A".format(current))

        # Calculate the force based on the induced current and magnetic field change rate
        #force_magnitude = induced_current * magnetic_field_change_rate

        # Calculate the average force in the XYZ components
        average_force = np.mean(np.abs(current))

        # Convert the accumulated energy to a color
        color_value = energy_to_color(average_force)

        simulate_oled_emission(average_force)

        print(color_value)
        screen.fill((255,255,255))
        pygame.draw.circle(screen, (color_value, 0, 0), (400, 300), 50)
        pygame.display.flip()
        
    else:    
        screen.fill((255,255,255))
        pygame.draw.circle(screen, (0, 0, 0), (400, 300), 50)
        pygame.display.flip()

    # Get the current position of the cone body
    current_position, _ = p.getBasePositionAndOrientation(cone_body)

    # Calculate the displacement from the previous position
    displacement = np.linalg.norm(np.array(current_position) - np.array(previous_position))

    # Accumulate the total distance
    total_distance += displacement

    # Update the previous position
    previous_position = current_position

    pygame.display.update()

    # Update PyBullet
    p.stepSimulation()

    frame_counter += 1

    # Add a sleep to control the simulation speed
    pygame.time.wait(1)
