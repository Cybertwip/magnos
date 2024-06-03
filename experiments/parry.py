import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Constants for recombination process
E_total = 4.184e15  # Energy in joules (1 megaton TNT equivalent)
m_p = 1.6726219e-27  # Mass of a proton in kg
m_e = 9.10938356e-31  # Mass of an electron in kg
c = 3e8  # Speed of light in m/s
eta = 0.0005  # Recombination efficiency
k = 1e-16  # Recombination rate constant
t_end = 10e-8  # Simulation end time in seconds (10 nanoseconds)

# Initial number of protons and electrons
N_p0 = E_total / (m_p * c**2)
N_e0 = E_total / (m_e * c**2)

# Define the differential equations for recombination and energy
def recombination_ode(y, t, eta, k, m_p, m_e, c):
    N_p, N_e, E = y
    dN_p_dt = -eta * k * N_p * N_e
    dN_e_dt = -eta * k * N_p * N_e
    dE_dt = -eta * k * N_p * N_e * (m_p * c**2 + m_e * c**2)
    return [dN_p_dt, dN_e_dt, dE_dt]

# Initial conditions vector for recombination
y0_recombination = [N_p0, N_e0, 0]

# Time points for the simulation
t = np.linspace(0, t_end, 1000)

# Integrate the system of ODEs for recombination
solution_recombination = odeint(recombination_ode, y0_recombination, t, args=(eta, k, m_p, m_e, c))

# Extract the results for recombination
N_p_solution = solution_recombination[:, 0]
N_e_solution = solution_recombination[:, 1]
E_recombination_solution = solution_recombination[:, 2]

# Constants for fission process
m_U235 = 3.9e-25  # Mass of a uranium-235 atom in kg

# Initial number of uranium-235 atoms
N_U2350 = E_total / (m_U235 * c**2)

# Define the differential equations for fission and energy release
def fission_ode(y, t, eta, k, m_U235, c):
    N_U235, E = y
    dN_U235_dt = -eta * k * N_U235**2
    dE_dt = -eta * k * N_U235**2 * (m_U235 * c**2)
    return [dN_U235_dt, dE_dt]

# Initial conditions vector for fission
y0_fission = [N_U2350, 0]

# Integrate the system of ODEs for fission
solution_fission = odeint(fission_ode, y0_fission, t, args=(eta, k, m_U235, c))

# Extract the results for fission
N_U235_solution = solution_fission[:, 0]
E_fission_solution = solution_fission[:, 1]

# Plot the results
plt.figure(figsize=(12, 6))

# Recombination plots
plt.subplot(2, 2, 1)
plt.plot(t, N_p_solution, label='Protons')
plt.plot(t, N_e_solution, label='Electrons')
plt.xlabel('Time (s)')
plt.ylabel('Number of Particles')
plt.title('Recombination: Number of Particles')
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, E_recombination_solution)
plt.xlabel('Time (s)')
plt.ylabel('Energy Released (J)')
plt.title('Recombination: Energy Released')

# Fission plots
plt.subplot(2, 2, 3)
plt.plot(t, N_U235_solution)
plt.xlabel('Time (s)')
plt.ylabel('Number of U-235 Atoms')
plt.title('Fission: Number of U-235 Atoms')

plt.subplot(2, 2, 4)
plt.plot(t, E_fission_solution)
plt.xlabel('Time (s)')
plt.ylabel('Energy Released (J)')
plt.title('Fission: Energy Released')

plt.tight_layout()
plt.show()
