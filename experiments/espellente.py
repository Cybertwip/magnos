import qutip as qt
import numpy as np
import matplotlib.pyplot as plt

def quantum_LC_circuit(N, w, times, g):
    """
    Simulate a quantum LC circuit with gravitatory-like repulsion.
    
    Parameters:
    - N: Number of quantum states to consider (dimension of the Hilbert space).
    - w: Natural frequency of the LC circuit (1/sqrt(LC)).
    - times: Times at which to compute the state.
    - g: Gravitatory-like repulsion coefficient.
    
    Returns:
    - result: Time evolution result of the LC circuit.
    """
    
    # Define creation and annihilation operators
    a = qt.destroy(N)
    
    # Hamiltonian for the LC circuit (using a quantum harmonic oscillator model)
    H = w * a.dag() * a + g * (a + a.dag())**4
    
    # Initial state: ground state
    psi0 = qt.basis(N, 0)
    
    # Evolve the state over time (no collapse operators for this closed system)
    result = qt.mesolve(H, psi0, times)
    
    return result

# Parameters
N = 10  # Number of quantum states
w = 1.0  # Frequency (in arbitrary units, set based on LC)
g = 0.1  # Gravitatory-like repulsion coefficient. Adjust as needed.
times = np.linspace(0, 4, 250)  # Span 4 hours, reduce to 250 points

# Simulate
result = quantum_LC_circuit(N, w, times, g)

# Extract probabilities for each state at each time step
probabilities = np.array([np.abs(psi.data.toarray())**2 for psi in result.states]).squeeze()

import numpy as np
import matplotlib.pyplot as plt

def distance_over_time(t, d0, v0, k, m):
    """
    Calculate distance between troposphere and Earth's surface over time.
    
    Parameters:
    - t: Time array.
    - d0: Initial distance.
    - v0: Initial velocity.
    - k: Gravitatory-like repulsion constant.
    - m: Mass of the troposphere.
    
    Returns:
    - d: Distance array over time.
    """
    a = k / m
    d = d0 + v0 * t + 0.5 * a * t**2
    return d

# Parameters
d0 = 10  # Initial distance, say 10 km (just as an example)
v0 = 0  # Let's assume it's initially at rest
k = 9.76e13  # Updated Gravitatory-like repulsion constant
m = 1e12  # Mass of the troposphere, a very rough estimate
t = np.linspace(0, 4, 250)  # Span 4 hours, reduce to 250 points

# Simulate
d = distance_over_time(t, d0, v0, k, m)

# Plot distance
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.plot(t, d)
plt.xlabel('Time (hours)')
plt.ylabel('Distance (km)')
plt.title('Distance between Troposphere and Earth with Gravitatory-like Repulsion')
plt.grid(True)

# Plot probabilities
plt.subplot(1, 2, 2)
for i in range(N):
    plt.plot(times, probabilities[:, i], label=f'State {i}')
plt.xlabel('Time')
plt.ylabel('Probability')
plt.title('LC Quantum Circuit Simulation with Gravitatory-like Repulsion')
plt.legend()

plt.tight_layout()
plt.show()
