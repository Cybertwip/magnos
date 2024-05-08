import numpy as np
import matplotlib.pyplot as plt

# Constants
mu0 = 4 * np.pi * 1e-7  # Permeability of free space
N_transmitter = 14  # Number of turns in the transmitter coil
N_receiver = 14  # Number of turns in each receiver coil
R = 1  # Radius of the coils in meters

# Simulation parameters
pulse_frequency = 1  # Frequency of the pulses in Hz
omega = 2 * np.pi * pulse_frequency  # Angular velocity of the transmitter

# Time array
t = np.linspace(0, 2, 1000)  # Simulate for 2 seconds with more resolution

# Function to calculate magnetic field at a distance z from the center of a loop
def magnetic_field(I, R, z):
    return (mu0 * I * R**2) / (2 * (R**2 + z**2)**(3/2))

# Function to simulate transmitter current as a function of time
def transmitter_current(t, omega):
    return np.sin(omega * t)  # Sine wave current with adjustable frequency

# Distances of receivers from the transmitter along the z-axis
z_distances = [0.5, 1.0, 1.5]  # Meters

# Calculate magnetic field at each receiver and induced voltage
voltages = []
for z in z_distances:
    B = magnetic_field(transmitter_current(t, omega), R, z)
    # Induced voltage in the receiver coil (Faraday's Law: V = -N*d(phi)/dt)
    V = -N_receiver * np.gradient(B, t)
    voltages.append(V)

# Plotting
plt.figure(figsize=(12, 8))
for i, V in enumerate(voltages):
    plt.plot(t, V, label=f'Receiver at {z_distances[i]} m (Frequency: {pulse_frequency} Hz)')
plt.title('Induced Voltage in Receiver Coils from a Simulated Pulsar with Adjustable Frequency')
plt.xlabel('Time (seconds)')
plt.ylabel('Voltage (volts)')
plt.legend()
plt.grid(True)
plt.show()
