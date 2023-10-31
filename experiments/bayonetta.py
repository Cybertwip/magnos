from qiskit import Aer, QuantumCircuit, transpile, execute
from qiskit.visualization import plot_histogram
import matplotlib.pyplot as plt

# Parameters
shots = 1000
rotation_angle = 3.14 / 2  # Small rotation angle for each iteration
iterations = 10
magnetic_field_angle =  3.14 / 2 # Initial magnetic field strength as an angle
margin_of_error = 0  # A small angle value that indicates the stopping criterion
decay_rate = 3.14 / 2  # Rate at which the magnetic field decays

# Create the quantum circuit
circuit = QuantumCircuit(2, 2)

# Create the superposition state
circuit.ry(3.14 / 2, 0)  # Start in a superposition

# Apply magnetic field rotations until the condition is met
while magnetic_field_angle != margin_of_error:
    circuit.ry(rotation_angle, 0)  # Applying the external rotation

    # Apply the current magnetic field
    circuit.rz(magnetic_field_angle, 0)
    # Reduce the magnetic field angle
    magnetic_field_angle -= decay_rate

# Apply SWAP gate to swap the states of q0 and q1
circuit.swap(0, 1)

# After applying all rotations, measure the final state
circuit.measure([0, 1], [0, 1])



# Execute the circuit
simulator = Aer.get_backend('qasm_simulator')
compiled_circuit = transpile(circuit, simulator)
result = execute(compiled_circuit, simulator, shots=shots).result()
counts = result.get_counts()

# Visualize
plot_histogram(counts)
plt.show()
