import numpy as np
from qutip import *
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def exponential_fit(t, a, b, c):
    return a * np.exp(b * t) + c

# Constants
N = 2
n_photons = 0
a = destroy(N)
H_field = a.dag() * a
psi0 = fock(N, n_photons)
total_time = np.pi 
num_time_steps = 10
times = np.linspace(0, total_time, num_time_steps)

# Iron-related properties
d = 1.0  # Example value for the dipole moment
mu_iron = 2.0  # Example value for the magnetic moment of iron in Bohr magnetons
c = 3e8  # Speed of light in meters per second

# Iron masses to simulate
iron_mass = 1

# Lists to store results
max_speeds_left = []
max_speeds_right = []
time_values = []


# Counter variables
time_counter_1 = 0
time_counter_2 = 0
time_counter_3 = 0


# Variables for time counter and polarity swapping
half_pi_time_reached = False
polarity_1l = 1  # Initial polarity
polarity_1r = 1  # Initial polarity
polarity_factor_1l = -1
polarity_factor_1r = 1


polarity_2l = 1  # Initial polarity
polarity_2r = 1  # Initial polarity
polarity_factor_2l = -1
polarity_factor_2r = 1

polarity_3l = 1  # Initial polarity
polarity_3r = 1  # Initial polarity
polarity_factor_3l = -1
polarity_factor_3r = 1

gravity_turn = 0

emf_left = []  # Store energy for the left iron bar
emf_right = []  # Store energy for the right iron bar

g = 9.81  # Acceleration due to gravity (m/s^2)

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    H_field = a.dag() * a
    H_left_iron = args['left_component_mass'] * (a.dag() + a)
    H_right_iron = args['right_component_mass'] * (a.dag() + a)
    return H_field + H_left_iron + H_right_iron

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_component_mass': iron_mass, 'right_component_mass': iron_mass}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args), options=Options(nsteps=10000))

# Calculate Casimir force values using a simple formula
emf_left_m_per_s_squared = (result.expect[0] * g)
emf_right_m_per_s_squared = (-result.expect[1] * g)


# Total time and number of time steps
total_time_pair_production = 1e-15 * np.pi 
num_time_steps_pair_production = 10
times_pair_production = np.linspace(0, total_time_pair_production, num_time_steps_pair_production)

index = 0
# Define the Hamiltonian for pair production with time-dependent EMF
def hamiltonian_pair_production_with_emf(t, args):
    global index
    omega = args['omega']  # Frequency of the incoming photon
    g = args['g']  # Coupling constant
    F_t = args['F_t']  # Time-dependent EMF function
    
    casimir_force = -np.pi / 2 * mu_iron * c / (240 * d**4) * F_t[index]
    index += 1

    # Hamiltonian terms
    H_photon = omega * a.dag() * a
    H_fermion = g * (a.dag() + a) * casimir_force
    return H_photon + H_fermion

# Set up the arguments for the Hamiltonian
hamiltonian_args_pair_production = {'omega': 2 * np.pi * 1e15, 'g': 1e-10, 'F_t': emf_left_m_per_s_squared}

# Initial state: vacuum state for the photon field
psi0_photon = fock(N, 0)

# Use sesolve to calculate the time-dependent results for pair production
result_pair_production_with_emf = sesolve(hamiltonian_pair_production_with_emf, psi0_photon,
                                           tlist=times_pair_production, args=(hamiltonian_args_pair_production),
                                           e_ops=[a.dag() * a, a.dag() + a],
                                           options=Options(nsteps=10000))

# Threshold for pair production
threshold = 1e-39
#pair_production_count = sum()
#print("Number of pair-produced particles:", pair_production_count)

# Plot the photon occupation number as a function of time
plt.plot(times_pair_production, result_pair_production_with_emf.expect[0] > threshold, marker='o')
plt.xlabel('Time (s)')
plt.ylabel('Pairs produced')
plt.title('Pair Production: Particle Presence vs. Time')
plt.show()
