import sys
import time
import pygame
import pybullet as p
import numpy as np
import poppy
import math

cfg = open("../pythia/Makefile.inc")
lib = "../pythia/lib"
for line in cfg:
    if line.startswith("PREFIX_LIB="): lib = line[11:-1]; break
sys.path.insert(0, lib)

class ParticleGenerator:
    def __init__(self):
        self.pythia = None
        self.particle_labels = {2212: "Proton", -2212: "Antiproton", 11: "Electron", -11: "Positron"}
        self.particle_counts = {2212: 0, -2212: 0, 11: 0, -11: 0}

    def initialize_pythia(self):
        import pythia8
        self.pythia = pythia8.Pythia()
        self.pythia.readString("Beams:idA = 2212")
        self.pythia.readString("Beams:idB = -2212")
        self.pythia.readString("Beams:eCM = 14.")
        self.pythia.readString("HardQCD:all = on")
        self.pythia.readString("WeakSingleBoson:ffbar2gmZ = on")
        self.pythia.readString("WeakSingleBoson:ffbar2W = on")
        self.pythia.readString("23:oneChannel = 1 1. 11 -11")
        self.pythia.readString("24:oneChannel = 1 1. 11 -12")
        self.pythia.readString("-24:oneChannel = 1 1. -11 12")

        if not self.pythia.init():
            print("Initialization failed!")
            exit()

    def generate_particles(self, num_events=10000):
        for iEvent in range(num_events):
            try:
                if not self.pythia.next():
                    continue
                for i in range(self.pythia.event.size()):
                    particle = self.pythia.event[i]
                    if particle.id() in self.particle_labels.keys():
                        self.particle_counts[particle.id()] += 1
            except Exception as e:
                print(f"Error: {e}")

    def print_particle_counts(self):
        for particle_id, count in self.particle_counts.items():
            print(f"Total {self.particle_labels[particle_id]}: {count}")

class MagnetSystem:
    def __init__(self):
        p.connect(p.DIRECT)
        p.setGravity(0, 0, -10)
        self.magnet_bodies = []
        self.total_torque = np.zeros(3)
        self.magnet_power = 0

    def create_magnets(self, positions):
        for position in positions:
            magnet_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
            magnet_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=magnet_shape, basePosition=position)
            self.magnet_bodies.append(magnet_body)
    
    def apply_magnetic_torque(self, alternator_body):
            _, angular_velocity = p.getBaseVelocity(alternator_body)
            angular_velocity = tuple(x + 0.00001 for x in angular_velocity)
            rotation_direction_vector = np.sign(angular_velocity)
            for i, magnet_body in enumerate(self.magnet_bodies):
                magnet_position, _ = p.getBasePositionAndOrientation(magnet_body)
                alternator_position, _ = p.getBasePositionAndOrientation(alternator_body)
                force_vector = np.array(magnet_position) - np.array(alternator_position)
                force_vector_normalized = force_vector / np.linalg.norm(force_vector)
                if np.dot(rotation_direction_vector, force_vector_normalized) > 0:
                    torque_magnitude = self.calculate_torque(MagnetSystem.magnet_volume, MagnetSystem.Magnetization_Neodymium, MagnetSystem.B_field_Iron)
                    torque_vector = torque_magnitude * force_vector_normalized
                    self.total_torque += torque_vector
                else:
                    torque_magnitude = 0
                    torque_vector = [0, 0, 0]
            p.applyExternalTorque(alternator_body, -1, self.total_torque, p.WORLD_FRAME)
            return self.total_torque

    @staticmethod
    def calculate_torque(magnet_volume, magnetization, B_field):
        return magnet_volume * magnetization * B_field

class AlternatorSystem:
    def __init__(self):
        self.alternator_body = None
        self.constraints = []

    def create_alternator(self):
        cone_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        self.alternator_body = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=cone_shape, basePosition=(0, 0, 2))

    def create_alternator_constraints(self, magnet_bodies):
        for magnet_body in magnet_bodies:
            constraint = p.createConstraint(parentBodyUniqueId=self.alternator_body, parentLinkIndex=-1,
                                            childBodyUniqueId=magnet_body, childLinkIndex=-1,
                                            jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0],
                                            parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.constraints.append(constraint)

class ElectricalSystem:
    c, m_e, m_p = 3.0e8, 9.11e-31, 1.67e-27
    E_pair = (m_e + m_p + m_e) * c**2

    NA = 6.022e23  # Avogadro's number
    hydrogen_atoms_per_liter = (NA / 22.4) * 2  # times 2 because H2 molecule has 2 H atoms

    hydrogen_atoms_0_1L = hydrogen_atoms_per_liter * 0.000001
    E_total_0001ML = E_pair * hydrogen_atoms_0_1L

    def __init__(self, alternator_system, magnet_system):
        self.alternator_system = alternator_system
        self.magnet_system = magnet_system
        self.efficiency = 0.1
        self.resistance, self.inductance = 1000.0, 0.1
        self.current, self.magnetic_field_change_rate = 0.0, 0.0
        self.total_energy, self.magnet_power = 0.0, 0
        self.prev_time = time.time()
        self.angular_velocity = 0
        self.electrical_power = 0
        self.time_difference = 0

    def apply_magnetic_torque_and_generate_energy(self):
        total_torque = self.magnet_system.apply_magnetic_torque(self.alternator_system.alternator_body)
        current_time = time.time()
        self.time_difference = current_time - self.prev_time
        mechanical_power = np.dot(total_torque, self.angular_velocity)
        electrical_power = mechanical_power * self.efficiency
        generated_energy = electrical_power * self.time_difference * self.efficiency
        self.total_energy += generated_energy
        self.magnet_power += generated_energy
        return total_torque

    def update_current_and_generate_energy(self, increase_power):
        if increase_power:
            energy_used = self.electrical_power * self.time_difference
        else:
            energy_used = 0.0
        required_input_power = energy_used / self.time_difference

        if increase_power:
            if (self.total_energy >= self.E_total_0001ML).any():
                increase_power = False
                if not enough_energy:
                    print("Accumulated Energy: {} J".format(total_energy))
                    print("Total Time Taken: {} seconds".format(total_time))
                    enough_energy = True
            else:
                electrical_system.apply_magnetic_torque_and_generate_energy()

class OLEDSimulation:
    def __init__(self):
        self.screen_width, self.screen_height = 800, 600
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.laser_shot, self.increase_power = False, False
        
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    self.laser_shot = True
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    self.laser_shot = False

    def draw_circle(self, color):
        self.screen.fill((255, 255, 255))
        pygame.draw.circle(self.screen, color, (400, 300), 50)
        pygame.display.flip()

    def update_display(self, increase_power, laser_shot):
        if laser_shot:
            if (total_energy > 0).any():
                total_energy = 0
                total_energy = 0.0
                total_torque = np.zeros(3)
                magnetic_field_change_rate = (magnet_power - current) / inductance
                induced_emf = -inductance * magnetic_field_change_rate
                induced_current = induced_emf / resistance
                current = magnet_power
                print("Estimated Electrical Power: {} W".format(electrical_power))
                print("Current: {} A".format(current))
                average_force = np.mean(np.abs(current))
                color_value = energy_to_color(average_force)
                simulate_oled_emission(average_force)
                print(color_value)
                self.draw_circle((color_value, 0, 0))
        else:
            if increase_power:
                self.draw_circle((0, 128, 0))
            else:
                self.draw_circle((0, 0, 0))

class SimulationManager:
    def __init__(self, particle_generator, magnet_system, alternator_system, electrical_system, oled_simulation):
        self.particle_generator = particle_generator
        self.magnet_system = magnet_system
        self.alternator_system = alternator_system
        self.electrical_system = electrical_system
        self.oled_simulation = oled_simulation
        self.previous_position = 0
        self.total_distance = 0

    def setup_simulation(self):
        self.particle_generator.initialize_pythia()
        positions = [((0, 1, 0), (0, 0, 1)), ((0, -1, 0), (0, 0, -1)),
                     ((-1, 0, 0), (1, 0, 0)), ((1, 0, 0), (-1, 0, 0)),
                     ((0, 0, 1), (0, -1, 0)), ((0, 0, -1), (0, 1, 0))]
        self.magnet_system.create_magnets(positions)
        self.alternator_system.create_alternator()
        self.alternator_system.create_alternator_constraints(self.magnet_system.magnet_bodies)

    def run_simulation(self, num_events=10000):
        self.particle_generator.generate_particles(num_events)
        self.particle_generator.print_particle_counts()

        for i in range(num_events):
            try:
                increase_power = True if i % 2 == 0 else False
                total_torque = self.electrical_system.apply_magnetic_torque_and_generate_energy()
                self.electrical_system.update_current_and_generate_energy(increase_power)
                self.oled_simulation.handle_events()
                self.oled_simulation.update_display(increase_power, self.oled_simulation.laser_shot)

                current_position, _ = p.getBasePositionAndOrientation(self.alternator_system.alternator_body)
                displacement = np.linalg.norm(np.array(current_position) - np.array(self.previous_position))
                self.total_distance += displacement
                self.previous_position = current_position

                pygame.display.update()
                p.stepSimulation()

                pygame.time.wait(1)
            except Exception as e:
                print(f"Error in simulation: {e}")

# Main
particle_generator = ParticleGenerator()
magnet_system = MagnetSystem()
alternator_system = AlternatorSystem()
electrical_system = ElectricalSystem(alternator_system, magnet_system)
oled_simulation = OLEDSimulation()
simulation_manager = SimulationManager(particle_generator, magnet_system, alternator_system, electrical_system, oled_simulation)

# Set up simulation
simulation_manager.setup_simulation()

# Run simulation
simulation_manager.run_simulation()
