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

p.setGravity(0, 0, -9.81)

# Define magnet positions (up, down, left, right, front, back)
magnet_positions = [
    ((0, 1, 0), (0, 0, 1)),
    ((0, -1, 0), (0, 0, -1)),
    ((-1, 0, 0), (1, 0, 0)),
    ((1, 0, 0), (-1, 0, 0)),
    ((0, 0, 1), (0, -1, 0)),
    ((0, 0, -1), (0, 1, 0)),
]

# Create the magnet objects as bullet cylinders
magnet_bodies = []
for position in magnet_positions:
    magnet_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.1])
    magnet_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=magnet_shape, basePosition=position[0], baseOrientation=position[1])
    magnet_bodies.append(magnet_body)


# Create the spaceship cone as a bullet box shape
cone_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
cone_body = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=cone_shape, basePosition=(0, 0, 2))

# Create the alternator object as a bullet sphere
alternator_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.01)
alternator_body = p.createMultiBody(baseMass=0.03298, baseCollisionShapeIndex=alternator_shape, basePosition=(0, 0, 0))

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


# Initialize variables for power and energy calculation
prev_time = time.time()
total_energy = 0.0
total_torque = np.zeros(3)  # Initialize total_torque here

def calculate_torque(magnet_volume, magnetization, B_field):
    return magnet_volume * magnetization * B_field


g = 9.81  # Gravitational acceleration on Earth in m/s^2

# Constants
mass_small_kg = 0.01  # Mass of the smaller object in kilograms

# Calculate the force required to lift the smaller object
force_small_required_N = mass_small_kg * g

# Input the current (in Amperes) and length (in meters) of the coil (for 60 kg)
current_A = 1  # Example current for the coil (for 60 kg)
length_m = 0.01  # Example length of the ferrous object (for 60 kg)

# Calculate the magnetic field strength (B) needed for the smaller object
magnetic_field_small_T = force_small_required_N / (current_A * length_m)

print(f"The required magnetic field strength to support {mass_small_kg} kg is approximately {magnetic_field_small_T:.2f} Tesla.")

# Calculate the magnetic field strength (B) for the Neodymium magnet
B_field_Neodymium = magnetic_field_small_T * 6  # Tesla (Typical value for Neodymium magnets)


num_alternators = 4

print(f"The equivalent magnetic field strength for the Neodymium magnet is approximately {B_field_Neodymium:.2f} Tesla.")


def time_to_stop(angular_velocity, torque, moment_of_inertia):
    # Calculate angular acceleration
    angular_acceleration = torque / moment_of_inertia
    
    # Calculate time to stop
    time_to_halt = -angular_velocity / angular_acceleration

    return time_to_halt

def energy_produced(angular_velocity, torque, angular_acceleration):
    """Calculate the energy produced by the alternator before it stops."""
    angular_displacement = abs(-(angular_velocity ** 2) / (2 * angular_acceleration))
    energy = abs(torque * angular_displacement)
    return energy


def apply_magnetic_torque(apply):
    # Reset total torque at the start
    total_torque = np.zeros(3)

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
        dot_product = np.dot(rotation_direction_vector, force_vector_normalized)
        if dot_product > 0:
            torque_magnitude = np.dot(force_vector_normalized, [0, 0, B_field_Neodymium])
            torque_vector = torque_magnitude * force_vector_normalized
            total_torque += torque_vector

    # Assuming you have the moment of inertia for your alternator
    moment_of_inertia = 1.3192e-6  # Calculated value for the 1 cm iron ball

    # Apply the total torque to the alternator
    if apply:
        p.applyExternalTorque(alternator_body, -1, total_torque, p.WORLD_FRAME)


    # Assuming angular_velocity is a scalar. If it's a vector, use its magnitude.
    angular_velocity_magnitude = np.linalg.norm(angular_velocity)


    # Calculate angular acceleration

    if apply:
        angular_acceleration = total_torque / moment_of_inertia  # Manually update angular velocity
    else:
        angular_acceleration = (total_torque / moment_of_inertia) * 2

    # Calculate energy produced 
    energy_output = energy_produced(angular_velocity_magnitude, total_torque, angular_acceleration) * num_alternators
    return total_torque, energy_output

increase_power = False

# Electrical constants
resistance = 1000.0  # Ohms (adjust this value as needed)
inductance = 0.1  # Henrys (adjust this value as needed)

# Initialize variables for current and magnetic field change rate
current = 0.0
magnetic_field_change_rate = 0.0

# Set up Pygame window
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption('OLED Brightness Simulation')

# Determine the number of hydrogen atoms
E_TOTAL_SLOW_CHARGER = 5
E_TOTAL_FAST_CHARGER = 29

# Target energy to be achieved before stopping
TARGET_ENERGY = E_TOTAL_SLOW_CHARGER

enough_energy = False

total_time = 0
stopping_time = 0

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.output_limits = output_limits

    def compute(self, error):
        # Proportional term
        p = self.kp * error

        # Integral term with anti-windup
        self.integral += error

        # Apply anti-windup to the integral term
        self.integral = np.clip(self.integral, self.output_limits[0] / self.ki, self.output_limits[1] / self.ki)

        # Derivative term
        d = error - self.prev_error

        # Compute output and clamp
        output = p + self.ki * self.integral + self.kd * d
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Save current error for the next iteration
        self.prev_error = error

        return output
    
# Initialize the PID controller with appropriate gains
pid_controller = PIDController(kp=1.0, ki=0.5, kd=0.1, output_limits=(-1, 1))

# Create a clock object to track time
clock = pygame.time.Clock()

adapter_hz = 79
p.setTimeStep(1.0 / adapter_hz)  # Set a higher time step for more simulation iterations

# Main loop
while True:
    # Update PyBullet
    p.stepSimulation()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            exit()

    dt = clock.tick(adapter_hz)

    keys = pygame.key.get_pressed()

    if keys[pygame.K_q]:
        increase_power = True
        # Calculate time difference since last frame
        current_time = time.time()
        time_difference = current_time - prev_time
        total_time += time_difference  # Accumulate time_difference to total_time
    else:
        increase_power = False

    # Calculate time difference since last frame
    current_time = time.time()
    time_difference = current_time - prev_time
    prev_time = current_time

    # Calculate the required input power to charge the magnets
    #required_input_power = energy_used / time_difference
    if increase_power:
        # Predict the expected energy by the time the alternator stops
        _, expected_next_energy = apply_magnetic_torque(False)
        
        # Compute the error for the PID controller
        error = TARGET_ENERGY - expected_next_energy
        # Compute the adjustment using the PID controller
        pid_output = pid_controller.compute(error)
        # Decide when to apply torque based on the PID output
        # For this case, let's only apply torque if pid_output is above a certain threshold.
        # This threshold can be fine-tuned based on your needs.
        if np.all(pid_output > 0.1):
            _, energy_output = apply_magnetic_torque(True)
            total_energy += energy_output

    # Apply the total torque to the alternator
    if increase_power:
        if (total_energy >= E_TOTAL_SLOW_CHARGER).any():
            print("Accumulated Energy: {} J".format(total_energy))
            print("Total Time Taken: {} seconds".format(total_time))

            screen.fill((255,255,255))
            pygame.draw.circle(screen, (255, 0, 0), (400, 300), 50)
            pygame.display.flip()

            total_energy = 0
            total_time = 0
            
    if increase_power:
        avg_energy = np.mean(total_energy)
        green_intensity = int((avg_energy / E_TOTAL_SLOW_CHARGER) * 255)

        green_intensity = max(0, green_intensity)

        screen.fill((255,255,255))
        pygame.draw.circle(screen, (0, green_intensity, 0), (400, 300), 50)
        pygame.display.flip()
    else:
        screen.fill((255,255,255))
        pygame.draw.circle(screen, (0, 0, 0), (400, 300), 50)
        pygame.display.flip()

    pygame.display.update()
