import numpy as np
import matplotlib.pyplot as plt
from qutip import *

# Define constants
N = 20  # Number of quantum states in the field mode
n_photons = 2  # Initial number of photons
left_iron_bar_strength = 4  # Strength of the left iron bar (arbitrary units)
right_iron_bar_strength = 5  # Strength of the right iron bar (arbitrary units)

# Create quantum operators
a = destroy(N)  # Annihilation operator for the field mode

# Define Hamiltonian for the field
H_field = a.dag() * a

# Total Hamiltonian (initially set to zero)
H_total = H_field

# Define initial state with n_photons in the field
psi0 = fock(N, n_photons)

# Time evolution
total_time = 1  # Total simulation time (adjust as needed)
num_time_steps = 100  # Number of time steps
times = np.linspace(0, total_time, num_time_steps)

# Initialize variables for changing distance between bars
initial_distance = 0.01  # Initial distance between the bars
final_distance = 0.00001  # Final distance between the bars

# Initialize lists to store EMF values for left and right bars
emf_left_values = []
emf_right_values = []

# Calculate time-dependent results
for t in times:
    d = initial_distance + (final_distance - initial_distance) * t / total_time
    H_left_iron = left_iron_bar_strength * (a.dag() + a) / d
    H_right_iron = -right_iron_bar_strength * (a.dag() + a) / d
    H_total = H_field + H_left_iron + H_right_iron  # Combine Hamiltonians
    result = mesolve(H_total, psi0, times, [], [a.dag() * a], options=Options(nsteps=100000))  # Use mesolve instead of sesolve

    emf = np.real(result.expect[0])  # EMF values
    emf_left_values.append(emf)  # Append EMF values for left bar
    emf_right_values.append(-emf)  # Append EMF values for right bar (negative for direction)

# Create separate plots for the left and right iron bars
plt.figure(1)
plt.plot(times, emf_left_values[-1], label='Left Iron Bar (EMF)')
plt.plot(times, emf_right_values[-1], label='Right Iron Bar (EMF)')
plt.xlabel('Time')
plt.ylabel('EMF Intensity')
plt.legend()

plt.show()
