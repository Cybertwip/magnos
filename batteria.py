import poppy
import numpy as np
import pybullet as p
import pygame
import math
import numpy as np
import time


class Boat_Settings:
    def __init__(self):
        # Pygame
        self.screen_width = 800
        self.screen_height = 600

        # Energy
        self.E_TOTAL_SLOW_CHARGER = 5
        self.E_TOTAL_FAST_CHARGER = 29
        self.E_TOTAL_BOAT_CHARGER = 87500
        self.TARGET_ENERGY = self.E_TOTAL_BOAT_CHARGER

        # PID
        self.kp = 1.44
        self.ki = 0.96
        self.kd = 0.54

        # Initialize PyBullet
        p.connect(p.DIRECT)

        p.setGravity(0, 0, -9.81)

        # PyBullet
        self.adapter_hz = 840
        p.setTimeStep(1.0 / self.adapter_hz)  

        # Alternator
        self.core_type = SiliconSteelBall
        self.core_radius = 0.05
        self.num_alternators = 5
        self.current_A = 5
        self.spins = 2500

class USB_C_Slow_Settings:
    def __init__(self):
        # Pygame
        self.screen_width = 800
        self.screen_height = 600

        # Energy
        self.E_TOTAL_SLOW_CHARGER = 5
        self.E_TOTAL_FAST_CHARGER = 29
        self.E_TOTAL_BOAT_CHARGER = 87500
        self.TARGET_ENERGY = self.E_TOTAL_SLOW_CHARGER

        # PID
        self.kp = 1.44
        self.ki = 0.96
        self.kd = 0.54

        # Initialize PyBullet
        p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)

        # PyBullet
        self.adapter_hz = 60
        p.setTimeStep(1.0 / self.adapter_hz)

        # Alternator
        self.core_type = IronBall
        self.core_radius = 0.01
        self.num_alternators = 2

        self.current_A = 0.01

        self.spins = 70


class USB_C_Fast_Settings:
    def __init__(self):
        # Pygame
        self.screen_width = 800
        self.screen_height = 600

        # Energy
        self.E_TOTAL_SLOW_CHARGER = 5
        self.E_TOTAL_FAST_CHARGER = 29
        self.E_TOTAL_BOAT_CHARGER = 87500
        self.TARGET_ENERGY = self.E_TOTAL_FAST_CHARGER

        # PID
        self.kp = 1.44
        self.ki = 0.96
        self.kd = 0.54

        # Initialize PyBullet
        p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)

        # PyBullet
        self.adapter_hz = 60
        p.setTimeStep(1.0 / self.adapter_hz)

        # Alternator
        self.core_type = IronBall
        self.core_radius = 0.01
        self.num_alternators = 2

        self.current_A = 0.01

        self.spins = 320     

pygame.init()
pygame.font.init()
font = pygame.font.SysFont(None, 25)  # Change 'None' to a font name if you have a preference.

# Initialize variables for power and energy calculation
prev_time = time.time()
total_energy = np.zeros(3)
total_torque = np.zeros(3)  # Initialize total_torque here
increase_power = False
total_time = 0
stopping_time = 0

def energy_produced(angular_velocity, torque, angular_acceleration):
    """Calculate the energy produced by the alternator before it stops."""
    angular_displacement = abs(-(angular_velocity ** 2) / (2 * angular_acceleration))
    energy = abs(torque * angular_displacement)
    return energy


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.output_limits = output_limits

    def compute(self, error, dt):
        # Proportional term
        p = self.kp * error

        # Integral term with consideration of dt
        self.integral += error * dt

        # Derivative term and consider dt
        d = (error - self.prev_error) / dt

        # Compute preliminary output
        output = p + self.ki * self.integral + self.kd * d
        
        # Anti-windup: If output is at its limit and error is still positive/negative, prevent further windup
        if (np.all(output > self.output_limits[1]) and np.all(error > 0)) or (np.all(output < self.output_limits[0]) and np.all(error < 0)):
            self.integral -= error * dt

        # Clamp output
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Save current error for the next iteration
        self.prev_error = error

        return output
    
# Create a clock object to track time
clock = pygame.time.Clock()

running = True

def update_display(screen, total_energy):
    global total_time, settings
    if (total_energy >= settings.TARGET_ENERGY).any():
        print("Accumulated Energy: {} J".format(total_energy))
        print("Total Time Taken: {} seconds".format(total_time))
        screen.fill((255, 255, 255))
        pygame.draw.circle(screen, (255, 0, 0), (400, 300), 50)
        pygame.display.flip()
        total_energy = np.zeros(3)
        total_time = 0
    else:
        avg_energy = np.mean(total_energy)
        green_intensity = int((avg_energy / settings.TARGET_ENERGY) * 255)
        green_intensity = max(0, green_intensity)
        screen.fill((255, 255, 255))
        pygame.draw.circle(screen, (0, green_intensity, 0), (400, 300), 50)
        pygame.display.flip()

    return total_energy



class MagneticBall:
    def __init__(self, radius):
        self.radius = radius
        self.shape = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        self.body = p.createMultiBody(baseMass=self.calculate_mass(), baseCollisionShapeIndex=self.shape, basePosition=(0, 0, 0))
        self.inertia = self.calculate_inertia()
        self.permeability = self.calculate_permeability()

    def calculate_volume(self):
        return (4/3) * math.pi * self.radius**3

    def calculate_mass(self):
        return self.density * self.calculate_volume()

    def calculate_inertia(self):
        return (2/5) * self.calculate_mass() * self.radius**2

    def calculate_permeability(self):
        return self.relative_permeability * 4 * math.pi * 1e-7  # Tm/A

class IronBall(MagneticBall):
    density = 7870  # kg/m^3
    relative_permeability = 5000  # This is a general value; actual value can vary widely

class PermalloyBall(MagneticBall):
    density = 8900  # kg/m^3 (approximate; this can vary)
    relative_permeability = 100000  # This is an approximate value for high-permeability permalloy

class SiliconSteelBall(MagneticBall):
    density = 7650  # kg/m^3
    relative_permeability = 3000  # This is a general value; actual value can vary

    
class Magnet:
    def __init__(self, core):
        # Define magnet positions (up, down, left, right, front, back)
        self.core = core
        self.positions = [
            ((0, 1, 0), (0, 0, 1)),
            ((0, -1, 0), (0, 0, -1)),
            ((-1, 0, 0), (1, 0, 0)),
            ((1, 0, 0), (-1, 0, 0)),
            ((0, 0, 1), (0, -1, 0)),
            ((0, 0, -1), (0, 1, 0)),
            # ((1, 1, 1), (-1, -1, -1)),  # Corner 1
            # ((1, 1, -1), (-1, -1, 1)),  # Corner 2
            # ((1, -1, 1), (-1, 1, -1)),  # Corner 3
            # ((-1, 1, 1), (1, -1, -1))  # Corner 4
        ]

        self.bodies = self.create_magnet_bodies()

        self.B_field_total_T = self.calculate_B_field()

    def create_magnet_bodies(self):
        magnet_bodies = []
        for position in self.positions:
            magnet_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.1])
            magnet_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=magnet_shape, basePosition=position[0], baseOrientation=position[1])
            magnet_bodies.append(magnet_body)
        return magnet_bodies
    
    def calculate_B_field(self):
        # Input the current (in Amperes) and length (in meters) of the coil
        current_A = settings.current_A
        n = settings.spins  # Number of turns per meter for the coil
        magnetic_field_coil_T = self.core.permeability * n * current_A
        num_coils = len(self.bodies)
        return num_coils * magnetic_field_coil_T


class Alternator:
    def __init__(self):
        self.core = SiliconSteelBall(0.05)
        self.magnet = Magnet(self.core)
        self.constraints = self.create_constraints()

    def create_constraints(self):
        constraints = []
        for magnet_body in self.magnet.bodies:
            constraint = p.createConstraint(
                parentBodyUniqueId=self.core.body,
                parentLinkIndex=-1,
                childBodyUniqueId=magnet_body,
                childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0],
            )
            constraints.append(constraint)
        return constraints


    def apply_magnetic_torque(self, apply):
        # Reset total torque at the start
        total_torque = np.zeros(3)

        # 1. Determine the rotation direction of the alternator
        _, angular_velocity = p.getBaseVelocity(self.core.body)

        # Add a small value to avoid zero division or zero magnitude
        angular_velocity = tuple(x + 0.00001 for x in angular_velocity)
        rotation_direction_vector = np.sign(angular_velocity)
    
        for i, magnet_body in enumerate(self.magnet.bodies):
            magnet_position, _ = p.getBasePositionAndOrientation(magnet_body)
            alternator_position, _ = p.getBasePositionAndOrientation(self.core.body)

            # Calculate the normalized force vector
            force_vector = np.array(magnet_position) - np.array(alternator_position)
            force_vector_normalized = force_vector / np.linalg.norm(force_vector)

            # Determine if the magnet should apply torque
            dot_product = np.dot(rotation_direction_vector, force_vector_normalized)
            if dot_product > 0:
                torque_magnitude = np.dot(force_vector_normalized, [0, 0, self.magnet.B_field_total_T])
                torque_vector = torque_magnitude * force_vector_normalized
                total_torque += torque_vector


        # Assuming you have the moment of inertia for your alternator
        moment_of_inertia = self.core.inertia

        # Apply the total torque to the alternator
        if apply:
            p.applyExternalTorque(self.core.body, -1, total_torque, p.WORLD_FRAME)

        # Assuming angular_velocity is a scalar. If it's a vector, use its magnitude.
        angular_velocity_magnitude = np.linalg.norm(angular_velocity)

        # Calculate angular acceleration
        angular_acceleration = total_torque / moment_of_inertia  # Manually update angular velocity

        # Calculate energy produced 
        energy_output = energy_produced(angular_velocity_magnitude, total_torque, angular_acceleration)
        return total_torque, energy_output

if __name__ == "__main__":
    settings = USB_C_Fast_Settings()

    # Set up Pygame window
    screen = pygame.display.set_mode((settings.screen_width, settings.screen_height))
    pygame.display.set_caption('OLED Brightness Simulation')

    pid_controller = PIDController(settings.kp, settings.ki, settings.kd, output_limits=(-1, 1))

    alternators = [Alternator() for _ in range(settings.num_alternators)]

    # Main loop
    while running:
        # Update PyBullet
        p.stepSimulation()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                exit()

        dt = clock.tick(60)
        keys = pygame.key.get_pressed()

        increase_power = keys[pygame.K_q]

        if increase_power:

            # Calculate time difference since last frame
            current_time = time.time()
            time_difference = current_time - prev_time
            prev_time = current_time
            total_time += time_difference  # Accumulate time_difference to total_time

            total_energy_output = 0
            total_expected_next_energy = 0
            
            for alt in alternators:
                _, expected_next_energy = alt.apply_magnetic_torque(False)
                total_expected_next_energy += expected_next_energy + total_energy

                # Compute the error for the PID controller
                error = settings.TARGET_ENERGY - total_expected_next_energy

                # Compute the adjustment using the PID controller
                pid_output = pid_controller.compute(error, dt / 1000.0)

                avg_pid_output = np.mean(pid_output)

                if np.all(avg_pid_output > 0):
                    _, energy_output = alt.apply_magnetic_torque(True)
                    total_energy_output += energy_output

            total_energy += total_energy_output


        total_energy = update_display(screen, total_energy)

        pygame.display.update()

