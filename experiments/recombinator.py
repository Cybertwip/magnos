import numpy as np
import matplotlib.pyplot as plt

# Constants
eta = 1e-3  # Recombination efficiency
k = 1e-2    # Constant
mp = 1.67e-27  # Mass of proton (kg)
me = 9.11e-31  # Mass of electron (kg)
c = 3e8     # Speed of light (m/s)

# Initial conditions
N0 = 1e20  # Initial number of loose protons and electrons
E_little_boy = 0.64 * 1e3 / 235 * 6.022e23 * 0.0138 * 200 * 1.60218e-13  # Initial energy from "Little Boy" (J)

# Time parameters
t_max = 1e-6  # Maximum time (s)
dt = 1e-9     # Time step (s)
num_steps = int(t_max / dt)

# Arrays to store results
t = np.linspace(0, t_max, num_steps)
N = np.zeros(num_steps)  # Number of loose protons/electrons
H = np.zeros(num_steps)  # Number of hydrogen atoms formed
E = np.zeros(num_steps)
dN_dt_arr = np.zeros(num_steps)

# Set initial values
N[0] = N0
H[0] = 0
E[0] = E_little_boy

# Euler integration
for i in range(1, num_steps):
    dN_dt = -eta * k * N[i-1] * N[i-1]  # Recombination rate
    dE_dt = -eta * k * N[i-1] * N[i-1] * (mp * c**2 + me * c**2)
    
    N[i] = N[i-1] + dN_dt * dt
    E[i] = E[i-1] + dE_dt * dt
    H[i] = H[i-1] - dN_dt * dt  # Increase in hydrogen atoms is negative change in loose particles
    dN_dt_arr[i] = dN_dt

# Plotting the results
plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(t, N, label='Number of Loose Protons/Electrons (N)')
plt.xlabel('Time (s)')
plt.ylabel('Number of Loose Protons/Electrons')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(t, H, label='Number of Hydrogen Atoms (H)', color='purple')
plt.xlabel('Time (s)')
plt.ylabel('Number of Hydrogen Atoms')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(t, E, label='Energy (E)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Energy (J)')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(t, dN_dt_arr, label='Recombination Rate (dN/dt)', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Recombination Rate')
plt.legend()

plt.tight_layout()
plt.show()
