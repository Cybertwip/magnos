import numpy as np
import matplotlib.pyplot as plt
from qutip import *

# Parameters
omega_nv = 2 * np.pi * 1.4e6  # NV center frequency in Hz
Bz = 1e-5  # Static magnetic field along the NV axis in Tesla
delta_omega = 2 * np.pi * 1e3  # Frequency of the external EMF in Hz
g_factor = 2.8e6  # Gyromagnetic ratio of NV center in Hz/T

# Function to define the time-dependent position of the magnet in meters
def magnet_position(t, max_distance=5, velocity=1):
    return max(0, max_distance - velocity * t)  # Start at max_distance meters and gradually decrease

# Hamiltonian terms for NV center
def hamiltonian(t, args):
    H0 = - g_factor * Bz * (spre(sigmap()) * spost(sigmam()) + spre(sigmam()) * spost(sigmap())) / 2
    H1 = delta_omega * magnet_position(t) * (spre(sigmax()) + spost(sigmax())) / 2  # External EMF along X-axis
    return H0 + H1

# Time evolution parameters
total_time = 1  # Total simulation time (adjust as needed)
num_time_steps = 200  # Number of time steps
times = np.linspace(0, total_time, num_time_steps)

# Initial state of the NV center
psi0 = basis(2, 0)  # Start in the ground state

# Use mesolve to calculate the time-dependent results
result = mesolve(hamiltonian, psi0, times, [], [sigmap(), sigmam(), sigmax()])

# Expectation values
expectation_sigmap = result.expect[0]
expectation_sigmam = result.expect[1]
expectation_sigmax = result.expect[2]

# Calculate the distance traveled over time
speed = np.real(expectation_sigmax)  # Using the sigma_x expectation value as speed
distance = np.cumsum(speed) * (times[1] - times[0])  # Cumulative sum to get distance

# Plotting
plt.figure(figsize=(12, 12))

# Plotting \langle\sigma_+\rangle vs. time
plt.subplot(3, 1, 1)
plt.plot(times, np.real(expectation_sigmap), label=r'$\langle\sigma_+\rangle$')
plt.xlabel('Time')
plt.ylabel('Expectation Value')
plt.legend()
plt.title('NV Center Response to External EMF')

# Plotting \langle\sigma_-\rangle vs. time
plt.subplot(3, 1, 2)
plt.plot(times, np.real(expectation_sigmam), label=r'$\langle\sigma_-\rangle$')
plt.xlabel('Time')
plt.ylabel('Expectation Value')
plt.legend()

# Plotting distance vs. time
plt.subplot(3, 1, 3)
plt.plot(times, distance, label='Distance Traveled')
plt.xlabel('Time')
plt.ylabel('Distance (meters)')
plt.legend()

plt.tight_layout()
plt.show()
