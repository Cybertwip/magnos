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
total_time = np.pi / 2 
num_time_steps = 10
times = np.linspace(0, total_time, num_time_steps)

# Iron-related properties
d = 1.0  # Example value for the dipole moment
mu_iron = 2.0  # Example value for the magnetic moment of iron in Bohr magnetons
c = 3e8  # Speed of light in meters per second

num_masses = 30
# Iron masses to simulate
iron_masses = np.linspace(1, num_masses, num_masses)  # Adjust as needed

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

energy_values_left = []  # Store energy for the left iron bar
energy_values_right = []  # Store energy for the right iron bar

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

    speed_l = speed_max_force_left
    speed_r = speed_max_force_right

    speed_l_equated = 0
    speed_r_equated = 0

    for i in range(num_time_steps):
        if time_counter_1 >= total_time / 2 and gravity_turn == 0:
            time_counter_1 = 0
            time_counter_2 = 0
            time_counter_3 = 0
            polarity_factor_1l *= -1
            polarity_factor_1r *= -1

            polarity_1l = polarity_factor_1l  # Swap polarity
            polarity_1r = polarity_factor_1r  # Swap polarity

            if gravity_turn == 0:
                gravity_turn = 1

        if time_counter_2 >= total_time / 8 and time_counter_2 <= (total_time / 8) * 3 and gravity_turn == 3:
            time_counter_1 = 0
            time_counter_2 = 0
            time_counter_3 = 0

            polarity_2l = polarity_factor_2l  # Swap polarity
            polarity_2r = polarity_factor_2r  # Swap polarity

            polarity_factor_2l *= -1
            polarity_factor_2r *= -1

            if gravity_turn == 1:
                gravity_turn = 2

        if time_counter_3 >= total_time / 8 and time_counter_3 <= (total_time / 8) * 3 and gravity_turn == 3:
            time_counter_1 = 0
            time_counter_2 = 0
            time_counter_3 = 0
            polarity_factor_3l *= -1
            polarity_factor_3r *= -1

            polarity_3l = polarity_factor_3l  # Swap polarity
            polarity_3r = polarity_factor_3r  # Swap polarity

            if gravity_turn == 2:
                gravity_turn = 0


        # Accelerate gravity offset
        if gravity_turn == 0:
            speed_l_equated -= speed_l * polarity_1l
            speed_r_equated -= speed_r * polarity_1r

        if gravity_turn == 1:
            speed_l_equated -= speed_l * polarity_2l
            speed_r_equated -= speed_r * polarity_2r

        if gravity_turn == 2:
            speed_l_equated -= speed_l * polarity_3l
            speed_r_equated -= speed_r * polarity_3r

        if gravity_turn == 3:
            speed_l_equated -= speed_l * polarity_1l
            speed_r_equated -= speed_r * polarity_1r


        if gravity_turn == 0:
            time_counter_1 += 1

        if gravity_turn == 1:
            time_counter_2 += 1

        if gravity_turn == 2:
            time_counter_3 += 1
            
        energy_values_left.append(speed_l**2 * hamiltonian_args['left_component_mass'] / 2)
        energy_values_right.append(speed_l**2 * hamiltonian_args['right_component_mass'] / 2)

        # Store the results
        max_speeds_left.append(np.abs(speed_l_equated))
        max_speeds_right.append(np.abs(speed_r_equated))

# Plot the results
plt.figure(figsize=(10, 10))

plt.subplot(4, 1, 1)
plt.plot(np.repeat(iron_masses, num_time_steps), max_speeds_left, label='Left Iron Bar')
plt.xlabel('Iron Mass')
plt.ylabel('Max Speed')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(np.repeat(iron_masses, num_time_steps), max_speeds_right, label='Right Iron Bar')
plt.xlabel('Iron Mass')
plt.ylabel('Max Speed')
plt.legend()

# Assuming power and frequency of the laser
power = 10000.0  # Adjust as needed, in watts
frequency = 30000.0  # Adjust as needed, in hertz

# Calculate charge based on power and frequency
charge = power / frequency

# Convert energy values to volts
volts_values_left = np.array(energy_values_left) / charge
volts_values_right = np.array(energy_values_right) / charge

# Plot energy values for the left iron bar
plt.subplot(4, 1, 3)
plt.plot(np.repeat(iron_masses, num_time_steps), volts_values_left, label='Right Iron Bar')
plt.xlabel('Time')
plt.ylabel('Energy (Volts)')
plt.legend()

# Plot energy values for the right iron bar
plt.subplot(4, 1, 4)
plt.plot(np.repeat(iron_masses, num_time_steps), volts_values_right, label='Right Iron Bar')
plt.xlabel('Time')
plt.ylabel('Energy (Volts)')
plt.legend()

plt.tight_layout()
plt.show()
