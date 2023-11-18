import numpy as np
from qutip import *
import matplotlib.pyplot as plt

# Constants
N = 2
n_photons = 0
a = destroy(N)
H_field = a.dag() * a
psi0 = fock(N, n_photons)
total_time = np.pi 
num_time_steps = 100
times = np.linspace(0, total_time, num_time_steps)

# Iron-related properties
d = 1.0  # Example value for the dipole moment
mu_iron = 2.0  # Example value for the magnetic moment of iron in Bohr magnetons
c = 3e8  # Speed of light in meters per second

# Iron masses to simulate
iron_masses = np.linspace(1, 10, 10)  # Adjust as needed

# Lists to store results
max_speeds_left = []
max_speeds_right = []

for iron_mass in iron_masses:
    # Define time-dependent Hamiltonian
    def hamiltonian_t(t, args):
        H_field = a.dag() * a
        H_left_iron = args['left_component_mass'] * (a.dag() + a)
        H_right_iron = args['right_component_mass'] * (a.dag() + a)
        return H_field + H_left_iron + H_right_iron

    # Set up the arguments for the Hamiltonian
    hamiltonian_args = {'left_component_mass': iron_mass, 'right_component_mass': iron_mass}

    # Use sesolve to calculate the time-dependent results
    result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args))

    # Calculate Casimir force values using a simple formula
    force_left_values = -np.pi / 2 * mu_iron * c / (240 * d**4) * np.gradient(np.real(result.expect[0]), times)
    force_right_values = -np.pi / 2 * mu_iron * c / (240 * d**4) * np.gradient(-np.real(result.expect[1]), times)

    # Find the time at which the force is maximized
    time_max_force_left = times[np.argmax(np.abs(force_left_values))]
    time_max_force_right = times[np.argmax(np.abs(force_right_values))]

    # Calculate the corresponding speed (assuming constant acceleration)
    speed_max_force_left = time_max_force_left * hamiltonian_args['left_component_mass']
    speed_max_force_right = time_max_force_right * hamiltonian_args['right_component_mass']

    # Store the results
    max_speeds_left.append(speed_max_force_left)
    max_speeds_right.append(speed_max_force_right)

# Plot the results
plt.figure(figsize=(10, 5))

plt.subplot(2, 1, 1)
plt.plot(iron_masses, max_speeds_left, label='Left Iron Bar')
plt.xlabel('Iron Mass')
plt.ylabel('Max Speed')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(iron_masses, max_speeds_right, label='Right Iron Bar')
plt.xlabel('Iron Mass')
plt.ylabel('Max Speed')
plt.legend()

plt.tight_layout()
plt.show()
