import pybullet as p
import pygame
import math
import numpy as np
import time

pygame.init()

# Initialize PyBullet
p.connect(p.GUI)
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

# Frame counter
frame_counter = 0

mouse_click = False 

# Electrical conversion efficiency (adjust this value as needed)
efficiency = 0.1

# Initialize variables for power and energy calculation
prev_time = time.time()
total_energy = 0.0
total_torque = np.zeros(3)  # Initialize total_torque here

# Magnets power level
magnet_power = 0

def calculate_torque(magnet_volume, magnetization, B_field):
    return magnet_volume * magnetization * B_field

# Example materials (these are just placeholders and should be replaced with actual values)
Magnetization_Neodymium = 1.2e6  # A/m (example value for neodymium magnet)
B_field_Iron = 0.002  # T (example value, assuming it gets magnetized by the magnet)

magnet_width = 0.1  # meters
magnet_length = 0.1  # meters
magnet_height = 0.1  # meters
magnet_volume = magnet_width * magnet_length * magnet_height * 6

def apply_magnetic_torque():
    global total_torque

    # 1. Determine the rotation direction of the alternator
    _, angular_velocity = p.getBaseVelocity(alternator_body)

    # Add a small value to avoid zero division or zero magnitude
    angular_velocity = tuple(x + 0.00001 for x in angular_velocity)
    rotation_direction_vector = np.sign(angular_velocity)

    for i, magnet_body in enumerate(magnet_bodies):
        magnet_position, _ = p.getBasePositionAndOrientation(magnet_body)
        alternator_position, _ = p.getBasePositionAndOrientation(alternator_body)

        # Calculate the normalized force vector
        force_vector = np.array(magnet_position) - np.array(alternator_position)
        force_vector_normalized = force_vector / np.linalg.norm(force_vector)

        # Determine if the magnet should apply torque
        if np.dot(rotation_direction_vector, force_vector_normalized) > 0:
            torque_magnitude = calculate_torque(magnet_volume, Magnetization_Neodymium, B_field_Iron)
            torque_vector = torque_magnitude * force_vector_normalized
            total_torque += torque_vector
        else:
            torque_magnitude = 0
            torque_vector = [0, 0, 0]

    # Apply the total torque to the alternator
    p.applyExternalTorque(alternator_body, -1, total_torque, p.WORLD_FRAME)

    return total_torque

space_ejection = False

# Electrical constants
resistance = 1.0  # Ohms (adjust this value as needed)
inductance = 0.1  # Henrys (adjust this value as needed)

# Initialize variables for current and magnetic field change rate
current = 0.0
magnetic_field_change_rate = 0.0

initial_position = np.array([0, 0, 0])
previous_position = initial_position
total_distance = 0.0

# Main loop
while True:
    # Check for mouse click manually using a keyboard key (e.g., 'a')
    keys = p.getKeyboardEvents()
    if ord('q') in keys and keys[ord('q')] == 1:
        # Increase magnet power on click
        mouse_click = True
    else:
        mouse_click = False 


    if ord('a') in keys and keys[ord('a')] == 1:
        # Increase magnet power on click
        space_ejection = True
    else:
        space_ejection = False 


    # Apply the total torque to the alternator and get the total torque value
    if mouse_click:
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

    if mouse_click:
        energy_used = electrical_power * time_difference
    else:
        energy_used = 0.0

    # Calculate the required input power to charge the magnets
    required_input_power = energy_used / time_difference

    # Print the estimated electrical power, accumulated energy, and required input power
    print("Estimated Electrical Power: {} W".format(electrical_power))
    print("Accumulated Energy: {} J".format(total_energy))
    #print("Required Input Power to Charge Magnets: {} W".format(required_input_power))

    # Apply the total torque to the alternator
    if mouse_click:
        apply_magnetic_torque()
            
    # Release space
    if space_ejection:
        #p.setGravity(0, 0, 0)
        
        # Calculate the change rate of magnetic field for the next frame
        magnetic_field_change_rate = (magnet_power - current) / inductance
        
        # Calculate the induced electromotive force (EMF) due to changing magnetic field
        induced_emf = -inductance * magnetic_field_change_rate

        # Calculate the induced current using Ohm's law (V = I * R)
        induced_current = induced_emf / resistance

        # Update the current value
        current = magnet_power

        # Calculate the force based on the induced current and magnetic field change rate
        force_magnitude = induced_current * magnetic_field_change_rate

        # Calculate the average force in the XYZ components
        average_force = np.mean(np.abs(force_magnitude))

        print("Induced Current:", induced_current)
        print("Force Magnitude:", force_magnitude)

        # Apply the force to the alternator body along the Z-axis
        p.applyExternalForce(cone_body, -1, [0, 0, average_force], [0, 0, 0], p.LINK_FRAME)

    # Get the current position of the cone body
    current_position, _ = p.getBasePositionAndOrientation(cone_body)

    # Calculate the displacement from the previous position
    displacement = np.linalg.norm(np.array(current_position) - np.array(previous_position))

    # Accumulate the total distance
    total_distance += displacement

    # Update the previous position
    previous_position = current_position

    # Print the total distance and the estimated speed
    print("Total Distance Traveled: {} units".format(total_distance))
    print("Estimated Speed: {} units/second".format(displacement / time_difference))


    # Update PyBullet
    p.stepSimulation()

    frame_counter += 1

    #p.getCameraImage(64, 64)  # Render to update GUI


    # Add a sleep to control the simulation speed
    pygame.time.wait(1)