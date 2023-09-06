import math


# Constants
G = 6.674e-11
#M = 5.972e24
M = 7.342e22 * 0.022
m = 1
R_EARTH = 173710
total_distance = 420
r_initial = R_EARTH + total_distance 
dt = 0.001

MAX_THRUST = 82260000

# Initialization
r = r_initial
v = -1500
a = 0

total_time = 0.0

# Simulate the descent
while r > R_EARTH:
    gravitational_force = G * M * m / r**2
    a_gravity = gravitational_force / m
    total_time += dt

    remaining_distance = r - R_EARTH
    required_deceleration = v**2 / (2 * remaining_distance)

    # Calculate the actual thrust required to get the required deceleration
    thrust_needed = (required_deceleration + a_gravity) * m

    # Adjust thrust dynamically, but don't exceed MAX_THRUST
    actual_thrust = min(thrust_needed, MAX_THRUST)
    
    a_thrust = actual_thrust / m
    a = a_gravity - a_thrust
    
    v += a * dt
    r += v * dt

    print(remaining_distance)
    print(r - R_EARTH)
    if r <= R_EARTH:
        v = 0

# Print the results
distance_fallen = r_initial - r
print(f"Distance Fallen: {distance_fallen:.2f} meters")
print(f"Final velocity: {v:.2f} m/s")
print(f"Total time: {total_time:.2f} seconds")
print(f"Error margin: {total_distance - distance_fallen:.2f} meters")
