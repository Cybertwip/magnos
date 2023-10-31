from qiskit import Aer, QuantumCircuit, transpile, execute
import qutip as qt
import matplotlib.pyplot as plt
import numpy as np


MAX_QUBITS = 20  # This should be a number high enough to accommodate all the qubits you expect to need

def prepare_superposition_qubits(energy, existing_circuit, current_qubit_index):
    num_new_qubits = int(energy * 10)  # Produce a number of qubits proportional to the energy

    # Calculate the maximum number of qubits that can be added
    max_new_qubits = MAX_QUBITS - current_qubit_index
    
    # Limit the number of new qubits to be added
    num_new_qubits = min(num_new_qubits, max_new_qubits)
    
    # Put these new qubits into a superposition state
    for _ in range(num_new_qubits):
        existing_circuit.h(current_qubit_index)
        current_qubit_index += 1

    return existing_circuit, current_qubit_index

def teleport_vacuum_plates():
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
    circuit, index = prepare_superposition_qubits(energy_from_main_loop, circuit, 0)

    # Execute the circuit
    simulator = Aer.get_backend('qasm_simulator')
    compiled_circuit = transpile(circuit, simulator)
    result = execute(compiled_circuit, simulator, shots=shots).result()
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

    # ... [rest of the code is omitted for clarity] ...

    return eigenvalues_oscillator, eigenvalues_with_boundary



def plot_eigenvalues(eigenvalues_oscillator, eigenvalues_with_boundary):
    x = list(range(len(eigenvalues_oscillator)))

    plt.figure(figsize=(10, 6))
    plt.plot(x, eigenvalues_oscillator, 'o-', label="Oscillator Eigenvalues")
    plt.plot(x, eigenvalues_with_boundary, 'x-', label="With Boundary Eigenvalues")
    
    plt.xlabel("Quantum Level")
    plt.ylabel("Eigenvalue")
    plt.title("Eigenvalues Comparison")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def calculate_gravitational_potential(frequency):
    # Constants
    h_bar = h / (2 * np.pi)  # Reduced Planck's constant, J*s
    G = 6.67430e-11  # Gravitational constant, m^3 kg^-1 s^-2
    c_squared = c**2  # Speed of light squared, m^2/s^2

    energy = h_bar * frequency

    # Calculate the effective mass associated with the gamma radiation energy
    effective_mass = energy / c_squared

    # Calculate the gravitational potential based on the effective mass
    gravitational_potential = G * effective_mass

    return gravitational_potential


# Constants
h = 6.62607015e-34  # Planck's constant, m^2 kg / s
c = 3e8  # Speed of light, m/s

def transition_frequencies(eigenvalues):
    delta_E = np.diff(eigenvalues)
    frequencies = delta_E / h
    return frequencies

def radiation_type(frequency):
    if frequency < 3e9:
        return "Radio waves"
    elif frequency < 3e11:
        return "Microwaves"
    elif frequency < 4.3e14:
        return "Infrared"
    elif frequency < 7.5e14:
        return "Visible light"
    elif frequency < 3e17:
        return "Ultraviolet"
    elif frequency < 3e19:
        return "X-rays"
    else:
        return "Gamma rays"

eigenvalues_oscillator, eigenvalues_with_boundary = teleport_vacuum_plates()  # Or False depending on where you want to teleport

frequencies_oscillator = transition_frequencies(eigenvalues_oscillator)
frequencies_with_boundary = transition_frequencies(eigenvalues_with_boundary)

intensities_oscillator = np.array([i for i in range(1, len(eigenvalues_oscillator))])  # Simplified intensity, proportional to n
intensities_with_boundary = np.array([i for i in range(1, len(eigenvalues_with_boundary))])

radiation_types_oscillator = [radiation_type(freq) for freq in frequencies_oscillator]
radiation_types_with_boundary = [radiation_type(freq) for freq in frequencies_with_boundary]



for i, (freq, intensity, rad_type) in enumerate(zip(frequencies_oscillator, intensities_oscillator, radiation_types_oscillator)):
    gravitational_potential = calculate_gravitational_potential(freq)
    print(f"Oscillator Transition {i} to {i+1}: Frequency = {freq:.2e} Hz, Intensity ~ {intensity}, Type = {rad_type}, Gravitational Potential = {gravitational_potential:.2e} J/kg")

print("\n")

for i, (freq, intensity, rad_type) in enumerate(zip(frequencies_with_boundary, intensities_with_boundary, radiation_types_with_boundary)):
    gravitational_potential = calculate_gravitational_potential(freq)
    print(f"Boundary Condition Transition {i} to {i+1}: Frequency = {freq:.2e} Hz, Intensity ~ {intensity}, Type = {rad_type}, Gravitational Potential = {gravitational_potential:.2e} J/kg")

print("Eigenvalues for the oscillator:", eigenvalues_oscillator)
print("Eigenvalues with boundary condition:", eigenvalues_with_boundary)

def calculate_gravitational_potential_from_energy(energy):
    # Constants
    G = 6.67430e-11  # Gravitational constant, m^3 kg^-1 s^-2
    c_squared = c**2  # Speed of light squared, m^2/s^2

    # Calculate the effective mass associated with the energy
    effective_mass = energy / c_squared

    # Calculate the gravitational potential based on the effective mass
    gravitational_potential = G * effective_mass

    return gravitational_potential

def calculate_gravitational_potential_from_gamma_energy(gamma_energy):
    # Constants
    G = 6.67430e-11  # Gravitational constant, m^3 kg^-1 s^-2
    c_squared = c**2  # Speed of light squared, m^2/s^2

    # Calculate the effective mass associated with the gamma energy
    effective_mass = gamma_energy / c_squared

    # Calculate the gravitational potential based on the effective mass
    gravitational_potential = G * effective_mass

    return gravitational_potential

# Constants
h = 6.62607015e-34  # Planck's constant, m^2 kg / s
c = 3e8  # Speed of light, m/s
G = 6.67430e-11  # Gravitational constant, m^3 kg^-1 s^-2

# Hydrogen atom properties
ionization_energy = 13.6  # Ionization energy of hydrogen atom, eV
transition_energy = 10.0   # Example energy for a transition, eV

# Calculate gamma energy
gamma_energy = transition_energy - ionization_energy

# Convert gamma energy to Joules
gamma_energy_joules = gamma_energy * 1.60218e-19  # Convert eV to Joules

# Calculate frequency of gamma radiation
gamma_frequency = gamma_energy_joules / h

# Calculate energy equivalent mass using E=mc^2
gamma_energy_mass = gamma_energy_joules / c**2

# Calculate gravitational potential due to gamma energy
gravitational_potential = G * gamma_energy_mass

print("Gamma energy:", gamma_energy_joules, "J")
print("Gamma frequency:", gamma_frequency, "Hz")
print("Gamma energy equivalent mass:", gamma_energy_mass, "kg")
print("Gravitational potential due to gamma energy:", gravitational_potential, "m^2/s^2")

def push_energy_to_negative_gravity():
    energy_from_main_loop = 100  # Initialize energy here
    
    # Initialize other variables
    shots = 1000

    rotation_angle = 3.14 / 2  
    magnetic_field_angle = 3.14 / 2
    margin_of_error = 0  
    decay_rate = 3.14 / 2 
    N = 10  
    w = 1.0  
    a = qt.destroy(N)

    current_qubit_index = 0  # Initialize the current qubit index

    gamma_energy_increment = 10  # Example increment value
    max_gamma_energy = 10000.0  # Example maximum gamma energy

    while gamma_energy_increment <= max_gamma_energy:
        
        # Recreate the quantum circuit
        circuit = QuantumCircuit(MAX_QUBITS, MAX_QUBITS)
        circuit.ry(3.14 / 2, 0)

        # Apply magnetic field rotations until the condition is met
        while magnetic_field_angle != margin_of_error:
            circuit.ry(rotation_angle, 0)
            circuit.rz(magnetic_field_angle, 0)
            magnetic_field_angle -= decay_rate

        # Limit the loop to the available qubit range
        #num_qubits = min(MAX_QUBITS, circuit.num_qubits)
        # for i in range(0, num_qubits - 1, 2):
        #     circuit.swap(i, i + 1)

        circuit.measure([0, 1], [0, 1])

        # Prepare superposition qubits
        circuit, current_qubit_index = prepare_superposition_qubits(energy_from_main_loop, circuit, current_qubit_index)
        simulator = Aer.get_backend('qasm_simulator')

        compiled_circuit = transpile(circuit, simulator)
        result = execute(compiled_circuit, simulator, shots=shots).result()
        counts = result.get_counts()

        if '0' in counts:
            boundary_condition = 0.1
        else:
            boundary_condition = 0.0

        H_with_boundary = w * a.dag() * a + boundary_condition * (a + a.dag())**2
        eigenvalues_with_boundary = H_with_boundary.eigenenergies()

        frequencies_with_boundary = transition_frequencies(eigenvalues_with_boundary)

        for i, freq in enumerate(frequencies_with_boundary):
            gamma_energy = gamma_energy_increment * 1.60218e-19  # Convert eV to Joules
            gravitational_potential = calculate_gravitational_potential_from_energy(gamma_energy)
            print(f"Energy Level = {energy_from_main_loop:.2f}, Transition {i} to {i+1}: Frequency = {freq:.2e} Hz, Gravitational Potential = {gravitational_potential:.2e} J/kg")
            print(f"Gamma energy = {gamma_energy_increment:.2f}")
            
            # Calculate potential energy using the gravitational potential
            potential_energy = -gravitational_potential  # Calculate potential energy
            print(f"Potential Energy = {potential_energy:.2e} J")
            
        gamma_energy_increment += 10  # Example increment step

# Call the function to push energy levels to negative gravity
push_energy_to_negative_gravity()

# Calling the function to plot the eigenvalues
plot_eigenvalues(eigenvalues_oscillator, eigenvalues_with_boundary)


