import matplotlib.pyplot as plt
import numpy as np 
import qutip as qt
from qiskit import QuantumCircuit, transpile
from qiskit_aer import QasmSimulator
import matplotlib.pyplot as plt

MAX_QUBITS = 31  # This should be a number high enough to accommodate all the qubits you expect to need

def prepare_superposition_qubits(energy, existing_circuit):
    num_new_qubits = int(energy * 10)  # Produce a number of qubits proportional to the energy
    
    # Starting qubit index to apply superposition. Assuming you're adding qubits sequentially.
    start_idx = existing_circuit.num_qubits - MAX_QUBITS
    
    # Put these new qubits into a superposition state
    for q in range(start_idx, start_idx + num_new_qubits):
        existing_circuit.h(q)

    return existing_circuit

# Parameters
shots = 1000
rotation_angle = 3.14 / 2  
iterations = 10
magnetic_field_angle =  3.14 / 2 
margin_of_error = 0  
decay_rate = 3.14 / 2  

# Create the quantum circuit
circuit = QuantumCircuit(MAX_QUBITS, MAX_QUBITS)

# Create the superposition state
circuit.ry(3.14 / 2, 0)

# Apply magnetic field rotations until the condition is met
while magnetic_field_angle != margin_of_error:
    circuit.ry(rotation_angle, 0)  # Applying the external rotation

    # Apply the current magnetic field
    circuit.rz(magnetic_field_angle, 0)
    # Reduce the magnetic field angle
    magnetic_field_angle -= decay_rate

for i in range(0, MAX_QUBITS - 1, 2):  # Step by 2 to get pairs
    circuit.swap(i, i + 1)

circuit.measure([0, 1], [0, 1])

# Now, integrate the superposition qubits into the circuit using our function
energy_from_main_loop = 2.5  # Example energy value, you'd update this from the main loop
circuit = prepare_superposition_qubits(energy_from_main_loop, circuit)

# Execute the circuit
simulator = QasmSimulator()
compiled_circuit = transpile(circuit, simulator)
job = simulator.run(compiled_circuit)
result = job.result()
counts = result.get_counts()

# If the teleportation was "successful" (just a simplification for this exercise)
if '0' in counts:
    boundary_condition = 0.1
else:
    boundary_condition = 0.0

# Parameters for our quantum system
N = 10  
w = 1.0  

# Creation and annihilation operators
a = qt.destroy(N)

# Hamiltonian for a single quantum oscillator
H_oscillator = w * a.dag() * a
eigenvalues_oscillator = H_oscillator.eigenenergies()

# Hamiltonian for a quantum oscillator with the boundary conditions from the teleportation
H_with_boundary = w * a.dag() * a + boundary_condition * (a + a.dag())**2
eigenvalues_with_boundary = H_with_boundary.eigenenergies()


# Decide on the gravitational potential based on the quantum outcome
if '0' in counts:
    gravitational_potential = 9.8  # example value, in m/s^2
else:
    gravitational_potential = 9.5  # slightly different gravitational potential

# Simulate a simple classical system affected by this gravitational potential using QuTiP

# Parameters for a simple harmonic oscillator in a gravitational field
m = 1  # mass
k = 1  # spring constant
g = gravitational_potential

# Hamiltonian representing a classical particle in a gravitational field
H_gravity = 0.5 * m * a.dag() * a - m * g * a.dag() + m * g * a

# Calculate the time evolution
times = np.linspace(0, 10, 100)
result = qt.mesolve(H_gravity, qt.basis(N, 0), times)

# Record the potential outcomes for plotting
outcomes = ['0', 'not 0']
potential_outcomes = []

if '0' in counts:
    potential_outcomes.append(9.8 + 0.2)
else:
    potential_outcomes.append(9.8)

if '0' not in counts:
    potential_outcomes.append(9.8 - 0.2)
else:
    potential_outcomes.append(9.8)
plt.figure(figsize=(8, 5))
plt.bar(outcomes, potential_outcomes, color=['blue', 'red'])
plt.xlabel('Teleportation Outcome')
plt.ylabel('Gravitational Potential (m/s^2)')
plt.title('Gravitational Potential based on Teleportation Outcome')
plt.ylim([9.5, 10])
for i, v in enumerate(potential_outcomes):
    plt.text(i, v + 0.01, str(v), ha='center', va='bottom', fontsize=10)
plt.tight_layout()


# Plot the position (expectation value of 'a') over time
plt.figure()
plt.plot(times, np.real(qt.expect(a, result.states)))
plt.xlabel("Time")
plt.ylabel("Position")
plt.title("Temporal Behavior due to Gravitational Potential")

# Plot the eigenvalues
plt.figure()
plt.plot(eigenvalues_oscillator, 'o-', label="Without Teleportation")
plt.plot(eigenvalues_with_boundary, 'x-', label="With Teleportation")
plt.xlabel("Eigenstate")
plt.ylabel("Energy")
plt.legend()
plt.title("Energy Levels of Quantum Oscillators")
plt.show()
