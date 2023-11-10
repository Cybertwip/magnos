import numpy as np
import matplotlib.pyplot as plt
from qutip import *

# Define constants
N = 2  # Number of quantum states in the field mode (1 photon or 0 photon)
n_photons = 0  # Initial number of photons (0 photon)

# Create quantum operators
a = destroy(N)  # Annihilation operator for the field mode

# Define Hamiltonian for the field
H_field = a.dag() * a

# Define initial state with n_photons in the field (0 photon)
psi0 = fock(N, n_photons)

# Time evolution
total_time = 3.14  # Total simulation time (adjust as needed)
num_time_steps = 200  # Number of time steps
times = np.linspace(0, total_time, num_time_steps)

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    # You can create a time-dependent Hamiltonian that transitions from 0 to 1 photon and back
    H_field = a.dag() * a
    H_left_iron = args['left_component_mass'] * (a.dag() + a)
    H_right_iron = args['right_component_mass'] * (a.dag() + a)
    return H_field + H_left_iron + H_right_iron

# Initialize lists to store EMF values for left and right bars
emf_left_values = []
emf_right_values = []

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_component_mass': 1.0, 'right_component_mass': 1.0}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args))

# Calculate EMF values
emf_left_values = np.real(result.expect[0])
emf_right_values = np.real(-result.expect[1])

# Create separate plots for the left and right iron bars with values
plt.figure(1)
plt.plot(times, emf_left_values, label='Left Iron Bar (EMF)')
plt.plot(times, emf_right_values, label='Right Iron Bar (EMF)')
plt.xlabel('Time')
plt.ylabel('EMF')
plt.legend()

g = 9.81  # Acceleration due to gravity (m/s^2)

# Calculate EMF values in m/s²
emf_left_m_per_s_squared = g - (emf_left_values * g) 
emf_right_m_per_s_squared = g - (emf_right_values * g)

# Create separate plots for the left and right iron bars with values in m/s²
plt.figure(2)
plt.plot(times, emf_left_m_per_s_squared, label='Left Iron Bar (EMF in m/s^2)')
plt.plot(times, emf_right_m_per_s_squared, label='Right Iron Bar (EMF in m/s^2)')
plt.xlabel('Time')
plt.ylabel('EMF (m/s^2)')
plt.legend()

# Calculate the offsets
gravity_offset = emf_left_m_per_s_squared - emf_right_m_per_s_squared

# Plot the offsets for the left and right iron bars
plt.figure(3)
plt.plot(times, gravity_offset * np.ones(len(times)), label='Gravitational offset')
plt.xlabel('Time')
plt.ylabel('Offset (m/s^2)')
plt.legend()

plt.show()

