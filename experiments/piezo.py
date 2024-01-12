import random

class IronLayerSimulator:
    def __init__(self, iron_thickness_mm, alternator_size_mm, iron_density, shake_frequency_hz):
        self.iron_thickness_mm = iron_thickness_mm
        self.alternator_size_mm = alternator_size_mm
        self.iron_density = iron_density
        self.shake_frequency_hz = shake_frequency_hz
        self.velocity = 0  # Initial velocity
        self.shake_count = 0  # Counter for the number of shakes

    def shake(self, duration_sec, resistance_ohms):
        total_electricity = 0
        for _ in range(int(duration_sec * self.shake_frequency_hz)):
            displacement_mm = random.uniform(0, 1)  # Random displacement during each shake
            velocity_change = 2 * displacement_mm * self.shake_frequency_hz  # Simplified formula
            self.velocity += velocity_change
            electricity_generated = self.generate_electricity()
            total_electricity += electricity_generated
            self.shake_count += 1

        current = total_electricity / duration_sec
        voltage = current * resistance_ohms
        return voltage


    def generate_electricity(self):
        # Simplified formula for electricity generation based on velocity
        electricity_generated = 0.5 * self.iron_thickness_mm * self.iron_density * self.velocity**2
        return electricity_generated

if __name__ == "__main__":
    iron_thickness_mm = 0.1
    alternator_size_mm = 0.01
    iron_density = 7.87  # Iron density in g/cm^3
    shake_frequency_hz = 2
    simulation_duration_sec = 5
    circuit_resistance_ohms = 0.05

    simulator = IronLayerSimulator(iron_thickness_mm, alternator_size_mm, iron_density, shake_frequency_hz)
    voltage_generated = simulator.shake(simulation_duration_sec, circuit_resistance_ohms)

    print(f"Voltage generated: {voltage_generated} Volts")
    print(f"Number of shakes: {simulator.shake_count}")
