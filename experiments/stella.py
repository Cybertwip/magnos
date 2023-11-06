from qiskit import Aer, QuantumCircuit, transpile, execute
import qutip as qt
import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame.locals import *

MAX_QUBITS = 1024
plate_effects = [0] * MAX_QUBITS
teleport_count = 0

# Define starship properties
starship_mass = 1000  # Example mass in kg
starship_position = 0  # Initial position
starship_velocity = 0  # Initial velocity

def prepare_superposition_qubits(energy, existing_circuit):
    num_new_qubits = int(energy * 10)  # Produce a number of qubits proportional to the energy
    
    # Starting qubit index to apply superposition. Assuming you're adding qubits sequentially.
    start_idx = existing_circuit.num_qubits - MAX_QUBITS
    
    # Put these new qubits into a superposition state
    for q in range(start_idx, start_idx + num_new_qubits):
        existing_circuit.h(q)

    return existing_circuit

def excite_iron_spin(circuit):
    # Assuming the second qubit represents the spin state of Iron
    theta = 3.14  # Hypothetical angle representing the magnetic field's effect on Iron's spin
    circuit.rz(theta, 1)  # Apply a magnetic field using Rz gate
    circuit.x(1)  # Spin-flip using a microwave pulse akin to Pauli-X gate
    return circuit

def gravitational_potential_to_speed(gravitational_potential):
    global teleport_count
    base_speed_range = 1.0
    potential_range = 9.8 - 3.5
    speed = -5 + (gravitational_potential - 3.5) * base_speed_range / potential_range
    
    speed_factor = 1 + teleport_count * 0.05
    speed *= speed_factor

    return speed

def move_starship(gravitational_potential):
    global starship_position, starship_velocity
    # Calculate gravitational force
    gravitational_force = starship_mass * gravitational_potential

    # Update starship velocity and position using F = ma
    starship_velocity += gravitational_force / starship_mass
    starship_position += starship_velocity



def teleport_vacuum_plates(to_origin: bool, position: float = 0.5):
    # Parameters
    global teleport_count
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

    gravitational_potential = 0.0
    # Decide on the gravitational potential based on the quantum outcome
    if to_origin:  # If we're teleporting back to Earth
        if '0' in counts:
            gravitational_potential = 9.8  # Gravitational potential for Earth
        else:
            gravitational_potential = 9.5  # slightly different gravitational potential for Earth
    else:  # If we're teleporting to Mars
        if '0' in counts:
            gravitational_potential = 3.7  # Gravitational potential for Mars
        else:
            gravitational_potential = 3.5  # slightly different gravitational potential for Mars
        
    # Adjust gravitational potential based on position
    gravitational_shift = position - 0.5  # center of screen is 0.5 in normalized coordinates

    # Before adjusting gravitational potential based on position
    gravitational_shift += sum(plate_effects)  # Adjust due to the effects of other plates

    gravitational_potential += gravitational_shift * 0.2 * teleport_count  # Adjust based on the number of teleported plates

    if to_origin:
        print("Vacuum plates teleported back")
    else:
        print(f"Vacuum plates teleported to target planet at y-position: {position}")

    if not to_origin:
        # After teleportation, selectively excite iron atoms
        circuit = excite_iron_spin(circuit)

        # Execute the circuit
        simulator = Aer.get_backend('qasm_simulator')
        compiled_circuit = transpile(circuit, simulator)
        result = execute(compiled_circuit, simulator, shots=1000).result()
        counts = result.get_counts()

    # Parameters for a simple harmonic oscillator in a gravitational field
    m = 1  # mass
    k = 1  # spring constant
    g = gravitational_potential

    # Hamiltonian representing a classical particle in a gravitational field
    H_gravity = 0.5 * m * a.dag() * a - m * g * a.dag() + m * g * a

    # Calculate the time evolution
    times = np.linspace(0, 10, 100)
    result = qt.mesolve(H_gravity, qt.basis(N, 0), times)    

    return gravitational_potential


def start_pygame_emergence_simulation(gravitational_potential):
    pygame.init()
    pygame.font.init()  # Initialize the font module
    
    font = pygame.font.SysFont(None, 25)  # Choose any font and size you like

    teleport_vacuum_plates(False)
    global teleport_count

    teleport_count = 0
    teleport_count += 1
    # Right after incrementing teleport_count:
    for i in range(len(plate_effects)):
        plate_effects[i] += 0.01  # For example, increase effect by 0.1

    WINDOW_WIDTH = 500
    WINDOW_HEIGHT = 500
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('Iron Emergence Simulation')

    WHITE = (255, 255, 255)
    BLUE = (0, 0, 255)
    BLACK = (0, 0, 0)

    clock = pygame.time.Clock()
    running = True

    quantumite_size = 100 if gravitational_potential < 9.8 else 50
    position_y = WINDOW_HEIGHT
    position_x = (WINDOW_WIDTH - quantumite_size) // 2
    emerged = False


    while running:
        screen.fill(WHITE)

        count_text = font.render(f"Teleport Count: {teleport_count}", True, BLACK)
        screen.blit(count_text, (10, 10))  # Adjust positioning as needed

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN :
                if event.key == K_LEFT:
                    gravitational_potential = teleport_vacuum_plates(False, position=(position_x - quantumite_size) / WINDOW_WIDTH)
                    teleport_count += 1
                    for i in range(len(plate_effects)):
                        plate_effects[i] += 0.01

                    # Update starship position based on gravitational potential
                    move_starship(gravitational_potential)

                elif event.key == K_RIGHT:
                    gravitational_potential = teleport_vacuum_plates(False, (position_x + quantumite_size) / WINDOW_WIDTH)
                    teleport_count += 1
                    for i in range(len(plate_effects)):
                        plate_effects[i] += 0.01

                    # Update starship position based on gravitational potential
                    move_starship(gravitational_potential)

                elif event.key == K_q:
                    for x in range(0, teleport_count):
                        gravitational_potential = teleport_vacuum_plates(True)
                        for i in range(len(plate_effects)):
                            plate_effects[i] -= 0.01

                    teleport_count = 0

        # Update starship's position
        position_x = int(WINDOW_WIDTH // 2)
        position_y = int(WINDOW_HEIGHT - starship_position - quantumite_size)

        pygame.draw.line(screen, BLACK, (0, WINDOW_HEIGHT // 2), (WINDOW_WIDTH, WINDOW_HEIGHT // 2), 2)
        pygame.draw.rect(screen, BLUE, (position_x, position_y, quantumite_size, quantumite_size))


        pygame.display.flip()
        clock.tick(60)


# Example starting gravitational potential
initial_gravitational_potential = 9.8
start_pygame_emergence_simulation(initial_gravitational_potential)
