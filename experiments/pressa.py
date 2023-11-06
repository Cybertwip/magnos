import numpy as np
import matplotlib.pyplot as plt
from qutip import *

# Define constants
N = 20  # Number of quantum states in the field mode
n_photons = 2  # Initial number of photons
left_iron_bar_strength = 0.2  # Strength of the left iron bar (arbitrary units)
right_iron_bar_strength = 0.5  # Strength of the right iron bar (arbitrary units)

# Create quantum operators
a = destroy(N)  # Annihilation operator for the field mode

# Define Hamiltonian for the field
H_field = a.dag() * a

# Hamiltonian for the left and right iron bars
H_left_iron = -left_iron_bar_strength * (a.dag() + a)
H_right_iron = right_iron_bar_strength * (a.dag() + a)  # Different sign for the right iron bar

# Total Hamiltonian
H_total = H_field + H_left_iron + H_right_iron

# Define initial state with n_photons in the field
psi0 = fock(N, n_photons)

# Time evolution
times = np.linspace(0, 1000, 100000)
result_left = mesolve(H_left_iron, psi0, times, [], [a.dag() * a])
result_right = mesolve(H_right_iron, psi0, times, [], [a.dag() * a])

# Convert photon number to EMF (assumption: photon number directly represents EMF intensity)
emf_left = np.real(result_left.expect[0])  # EMF value for the left iron bar
emf_right = np.real(result_right.expect[0])  # EMF value for the right iron bar

# Plot the results for both Iron bars
plt.plot(times, emf_left, label='Left Iron Bar (EMF)')
plt.plot(times, emf_right, label='Right Iron Bar (EMF)')
plt.xlabel('Time')
plt.ylabel('EMF Intensity')
plt.legend()
plt.show()
