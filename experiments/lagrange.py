import numpy as np
import matplotlib.pyplot as plt

# Gravitational constant (m^3 kg^-1 s^-2)
G = 6.67430e-11

# Constants for masses of celestial bodies (in kg)
M_mercury = 3.285e23
M_venus = 4.867e24
M_earth = 5.972e24
M_mars = 6.39e23

# Mass of Falcon 9 rocket (kg)
M_falcon9 = 549054  # Mass of Falcon 9 (kg)

# Launch distance from Earth to Mars (m)
launch_distance = 225e9

# Distances from the rocket to the celestial bodies (in meters)
r_mercury = 0.39 * 1.496e11  # Distance to Mercury (average distance from the Sun to Mercury in meters)
r_venus = 0.72 * 1.496e11  # Distance to Venus (average distance from the Sun to Venus in meters)
r_earth = 1.0 * 1.496e11  # Distance to Earth (average distance from the Sun to Earth in meters)
r_mars = 1.52 * 1.496e11  # Distance to Mars (average distance from the Sun to Mars in meters)

# Calculate gravitational forces relative to Earth
F_values = []
celestial_bodies = []

for r, M, body in zip([r_mercury - r_earth, r_venus - r_earth, launch_distance, r_mars - r_earth],
                      [M_mercury, M_venus, M_earth, M_mars],
                      ["Mercury", "Venus", "Earth", "Mars"]):
    F_values.append(G * M_falcon9 * M / r**2)
    celestial_bodies.append(body)

# Plot gravitational forces
plt.bar(celestial_bodies, F_values)
plt.xlabel('Celestial Body')
plt.ylabel('Gravitational Force (N)')
plt.title('Gravitational Forces Relative to Earth on a Falcon 9 Rocket Trajectory to Mars')
plt.grid(axis='y')
plt.show()
