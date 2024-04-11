import numpy as np
import matplotlib.pyplot as plt

class SolarSail:
    def __init__(self, area):
        self.area = area  # Area of the sail in square meters

    def calculate_pressure_casimir(self, distance, time):
        # Constants
        casimir_constant = 6.626e-34  # Planck's constant in m^2 kg / s
        speed_of_light = 299792458  # Speed of light in m/s
        pi = np.pi

        # Calculate pressure exerted by Casimir effect using the provided equation
        frequency = 3000000  # Frequency of pulsation in Hz
        amplitude = 1e-2  # Amplitude of pulsation
        pulsation = amplitude * np.sin(2 * pi * frequency * time)
        
        pressure = (pi**2 * casimir_constant * self.area) / (240 * (distance**4 * speed_of_light**3)) + pulsation
        return pressure

    def calculate_acceleration(self, pressure, mass):
        # Calculate acceleration using F = ma
        acceleration = pressure / mass
        return acceleration

# Function to calculate distance between Earth and Mars at a given time
def distance_earth_to_mars(time_days):
    # Your function to calculate distance goes here
    # Example: Assuming linear interpolation between the closest approach and the furthest distance
    closest_approach = 54.6e6  # Closest approach distance in kilometers
    furthest_distance = 401.3e6  # Furthest distance in kilometers
    time_period = 687  # Orbital period of Mars in Earth days
    distance = closest_approach + (furthest_distance - closest_approach) * (time_days / time_period)
    return distance * 1000  # Convert to meters

# Time array for the trip (in Earth days)
time_days = np.linspace(0, 20, 100)  # Assuming a full Martian year for the trip

# Create a SolarSail object with a given area
sail = SolarSail(area=10)  # Assuming a sail area of 10 square meters

# Calculate pressure exerted by Casimir effect on the sail at each time point
pressures = []
for t in time_days:
    distance = distance_earth_to_mars(t)
    pressure = sail.calculate_pressure_casimir(distance, t)
    pressures.append(pressure)

# Calculate spaceship speed using the pressure and mass of the spaceship
mass_spaceship = 1000  # Mass of the spaceship in kg
accelerations = [sail.calculate_acceleration(p, mass_spaceship) for p in pressures]
velocities = np.cumsum(accelerations) * (time_days[1] - time_days[0]) * 86400  # Convert days to seconds

# Integrate velocity to calculate distance traveled
distances = np.cumsum(velocities) * (time_days[1] - time_days[0]) * 86400  # Convert days to seconds

# Plot pressure, spaceship speed, and distance traveled
fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Time (Earth days)')
ax1.set_ylabel('Pressure (Pa)', color=color)
ax1.plot(time_days, pressures, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  
color = 'tab:blue'
ax2.set_ylabel('Speed (m/s)', color=color)  
ax2.plot(time_days, velocities, color=color)
ax2.tick_params(axis='y', labelcolor=color)

ax3 = ax1.twinx()  
color = 'tab:green'
ax3.spines['right'].set_position(('outward', 60))
ax3.set_ylabel('Distance Traveled (m)', color=color)
ax3.plot(time_days, distances, color=color)
ax3.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  
plt.title('Casimir Forces Pressure on Solar Sail, Spaceship Speed, and Distance Traveled during Trip from Earth to Mars')
plt.grid(True)
plt.show()
