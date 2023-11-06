import numpy as np
import matplotlib.pyplot as plt
from qutip import *

# Define constants
N = 20  # Number of quantum states in the field mode
n_photons = 2  # Initial number of photons

# Create quantum operators
a = destroy(N)  # Annihilation operator for the field mode

# Define Hamiltonian for the field
H_field = a.dag() * a

# Define initial state with n_photons in the field
psi0 = fock(N, n_photons)

# Time evolution
total_time = 1  # Total simulation time (adjust as needed)
num_time_steps = 100  # Number of time steps
times = np.linspace(0, total_time, num_time_steps)

# Initialize variables for changing distance between bars
initial_distance = 0.01  # Initial distance between the bars
final_distance = 0.001  # Final distance between the bars

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    d = initial_distance + (final_distance - initial_distance) * t / total_time
    H_left_iron = args['left_iron_bar_strength'] * (a.dag() + a) / d # Swap the operators here
    H_right_iron = args['right_iron_bar_strength'] * (a.dag() + a) / d # Swap the operators here
    return H_field + H_left_iron + H_right_iron

# Initialize lists to store EMF values for left and right bars
emf_left_values = []
emf_right_values = []

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_iron_bar_strength': 1.0, 'right_iron_bar_strength': 2.0}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args)) # Swap the operators here

# Calculate EMF values
emf_left_values = np.real(result.expect[0])
emf_right_values = np.real(-result.expect[1])

# Create separate plots for the left and right iron bars
plt.figure(1)
plt.plot(times, emf_left_values, label='Left Iron Bar (EMF Loss)')
plt.plot(times, emf_right_values, label='Right Iron Bar (EMF Gain)')
plt.xlabel('Time')
plt.ylabel('EMF Intensity')
plt.legend()

plt.show()
