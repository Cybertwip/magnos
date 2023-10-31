import qutip as qt
import numpy as np
import matplotlib.pyplot as plt

def quantum_LC_circuit(N, w, times):
    """
    Simulate a quantum LC circuit.
    
    Parameters:
    - N: Number of quantum states to consider (dimension of the Hilbert space).
    - w: Natural frequency of the LC circuit (1/sqrt(LC)).
    - times: Times at which to compute the state.
    
    Returns:
    - result: Time evolution result of the LC circuit.
    """
    
    # Define creation and annihilation operators
    a = qt.destroy(N)
    
    # Hamiltonian for the LC circuit (using a quantum harmonic oscillator model)
    H = w * a.dag() * a
    
    # Initial state: ground state
    psi0 = qt.basis(N, 0)
    
    # Evolve the state over time (no collapse operators for this closed system)
    result = qt.mesolve(H, psi0, times)
    
    return result

# Parameters
N = 10  # Number of quantum states
w = 1.0  # Frequency (in arbitrary units, set based on LC)
times = np.linspace(0, 10, 500)

# Simulate
result = quantum_LC_circuit(N, w, times)

# Extract probabilities for each state at each time step
probabilities = np.array([np.abs(psi.data.toarray())**2 for psi in result.states]).squeeze()

# Plot the probabilities
for i in range(N):
    plt.plot(times, probabilities[:, i], label=f'State {i}')

plt.xlabel('Time')
plt.ylabel('Probability')
plt.title('LC Quantum Circuit Simulation')
plt.legend()
plt.show()
