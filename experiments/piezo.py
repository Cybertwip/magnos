import pygame
import random
import sys

class IronLayerSimulator:
    def __init__(self, iron_thickness_mm, alternator_size_mm, iron_density, shake_frequency_hz):
        self.iron_thickness_mm = iron_thickness_mm
        self.alternator_size_mm = alternator_size_mm
        self.iron_density = iron_density
        self.shake_frequency_hz = shake_frequency_hz
        self.velocity = 0  # Initial velocity
        self.shake_count = 0  # Counter for the number of shakes

    def shake(self, duration_sec, resistance_ohms):
        total_photon_energy = 0
        displacement_mm = random.uniform(0, 1)  # Random displacement during each shake
        force = self.calculate_force(displacement_mm)
        velocity_change = force / (0.5 * self.iron_density * self.iron_thickness_mm)  # Simplified formula
        self.velocity = velocity_change
        energy_generated = self.generate_energy(self.shake_frequency_hz)
        total_photon_energy += energy_generated
        self.shake_count += 1

        current = total_photon_energy / duration_sec
        voltage = current * resistance_ohms

        return voltage, self.velocity

    def photonize(self, duration_sec, photon_frequency_thz, resistance_ohms):
        total_photon_energy = 0
        displacement_mm = random.uniform(0, 1)  # Random displacement during each shake
        force = self.calculate_force(displacement_mm)
        velocity_change = force / (0.5 * self.iron_density * self.iron_thickness_mm)  # Simplified formula
        self.velocity = velocity_change
        energy_generated = self.generate_photonic_energy(photon_frequency_thz)
        total_photon_energy += energy_generated
        self.shake_count += 1

        current = total_photon_energy / duration_sec
        voltage = current * resistance_ohms

        return voltage, self.velocity
    

    def calculate_force(self, displacement_mm):
        # Harmonic oscillator force equation: F(x) = -m * omega^2 * x
        omega = 2 * self.shake_frequency_hz * 3.14159  # Convert shake frequency to angular frequency
        force = -self.iron_density * omega**2 * displacement_mm * 0.001  # Convert displacement to meters
        return force

    def generate_energy(self, frequency_hz):
        # Simplified formula for energy generation based on velocity
        energy_generated = 0.5 * self.iron_thickness_mm * self.iron_density * self.velocity**2
        # Convert energy to photon energy using E = h * f
        photon_energy = energy_generated / frequency_hz  # Convert Hz to THz
        return photon_energy

    def generate_photonic_energy(self, photon_frequency_thz):
        # Simplified formula for energy generation based on velocity
        energy_generated = 0.5 * self.iron_thickness_mm * self.iron_density * self.velocity**2
        # Convert energy to photon energy using E = h * f
        photon_energy = energy_generated * 1e12 / photon_frequency_thz  # Convert Hz to THz
        return photon_energy

# Pygame setup
pygame.init()
clock = pygame.time.Clock()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Iron Layer Simulator")

# Colors
iron_color = (128, 128, 128)
brightness_color = (255, 255, 0)

def draw_simulation(screen, iron_thickness_mm, velocity, shake_count, voltage, brightness_radius):
    screen.fill((255, 255, 255))

    # Draw iron layer
    pygame.draw.rect(screen, iron_color, (width / 2 - iron_thickness_mm / 2, height / 2 - 50, iron_thickness_mm, 100))

    # Draw alternators (for visualization purposes)
    alternator_size = 5
    for i in range(int(iron_thickness_mm / alternator_size)):
        pygame.draw.rect(screen, (0, 0, 255), (width / 2 - iron_thickness_mm / 2 + i * alternator_size, height / 2 - alternator_size / 2, alternator_size, alternator_size))

    # Draw yellow circle representing brightness
    pygame.draw.circle(screen, brightness_color, (width / 2, height / 2), brightness_radius)

    # Display simulation information
    font = pygame.font.Font(None, 36)
    text = font.render(f"Layer Shake Speed: {velocity:.10f} mm/s", True, (0, 0, 0))
    screen.blit(text, (20, 20))

    text = font.render(f"Shake Count: {shake_count}", True, (0, 0, 0))
    screen.blit(text, (20, 60))

    text = font.render(f"Voltage: {voltage:.2f} Volts", True, (0, 0, 0))
    screen.blit(text, (20, 100))

    pygame.display.flip()

# Simulation parameters
iron_thickness_mm = 0.1  # Reduced thickness for better visualization
alternator_size_mm = 0.01
iron_density = 7.87  # Standard iron density in g/cm^3
shake_frequency_hz = 60
simulation_duration_sec = 5
circuit_resistance_ohms = 0.005  # Adjusted circuit resistance to 5 ohms

simulator = IronLayerSimulator(iron_thickness_mm, alternator_size_mm, iron_density, shake_frequency_hz)

iron_thickness_mm = 1000
alternator_size_mm = 1000
iron_density = 1.0  # Reduced iron density in g/cm^3
shake_frequency_hz = 5
simulation_duration_sec = 5
photon_frequency_thz = 500  # Example photon frequency in THz
circuit_resistance_ohms = 0.005  # Adjusted circuit resistance to 5 ohms

# Create simulator instance
photon_simulator = IronLayerSimulator(iron_thickness_mm, alternator_size_mm, iron_density, shake_frequency_hz)

# Buttons
shake_button_rect = pygame.Rect(20, 150, 100, 50)
photonize_button_rect = pygame.Rect(150, 150, 150, 50)

voltage = 0
velocity = 0
brightness_radius = 0

buffer = pygame.Surface((width, height))


# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if shake_button_rect.collidepoint(event.pos):
                    voltage, velocity = simulator.shake(simulation_duration_sec, circuit_resistance_ohms)
                    brightness_radius = int((voltage / 100.0) * 50)  # Adjust 1000.0 to match the maximum voltage

                elif photonize_button_rect.collidepoint(event.pos):

                    brightness_radius = int((voltage / 1000.0) * 50)  # Adjust 1000.0 to match the maximum voltage

                    voltage, velocity = photon_simulator.photonize(simulation_duration_sec, photon_frequency_thz, circuit_resistance_ohms)

    # Draw the simulation
    draw_simulation(buffer, iron_thickness_mm, abs(velocity), simulator.shake_count, voltage, brightness_radius)

    # Draw the buffer to the screen
    screen.blit(buffer, (0, 0))

    # Draw buttons
    pygame.draw.rect(screen, (0, 255, 0), shake_button_rect)
    pygame.draw.rect(screen, (0, 255, 0), photonize_button_rect)
    font = pygame.font.Font(None, 36)
    text = font.render("Shake", True, (0, 0, 0))
    screen.blit(text, (shake_button_rect.x + 10, shake_button_rect.y + 10))
    text = font.render("Photonize", True, (0, 0, 0))
    screen.blit(text, (photonize_button_rect.x + 10, photonize_button_rect.y + 10))

    pygame.display.flip()
    clock.tick(60)


pygame.quit()
sys.exit()
