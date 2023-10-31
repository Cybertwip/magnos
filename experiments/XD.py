import pygame
from qiskit import Aer, QuantumCircuit, transpile, execute

# Convert string to binary
def string_to_binary(s):
    return ''.join(format(ord(c), '08b') for c in s)

def binary_to_string(binary_str):
    return ''.join(chr(int(binary_str[i:i+8], 2)) for i in range(0, len(binary_str), 8))

# Quantum function to encode and decode a given character
def quantum_encode_decode(char_input):
    binary_hello_world = string_to_binary(char_input)

    # Parameters
    shots = 1000
    rotation_angle = 3.14 / 2  
    magnetic_field_angle =  3.14 / 2 
    margin_of_error = 0  
    decay_rate = 3.14 / 2  

    # Create the quantum circuit
    num_qubits = len(binary_hello_world)
    circuit = QuantumCircuit(num_qubits, num_qubits)

    reversed_binary_hello_world = binary_hello_world[::-1]
    # We're directly encoding our binary string onto the qubits
    for i, bit in enumerate(reversed_binary_hello_world):
        if bit == '1':
            circuit.x(i)

    # Create the superposition state
    for qubit_index in range(num_qubits):
        circuit.ry(3.14 / 2, qubit_index)

    # Apply magnetic field rotations until the condition is met
    while magnetic_field_angle != margin_of_error:
        for qubit_index in range(num_qubits):
            circuit.ry(rotation_angle, qubit_index)  # Applying the external rotation
            circuit.rz(magnetic_field_angle, qubit_index)  # Apply the current magnetic field
        
        # Reduce the magnetic field angle
        magnetic_field_angle -= decay_rate

    for i in range(0, num_qubits - 1, 2):  # Step by 2 to get pairs
        circuit.swap(i, i + 1)

    for i in range(num_qubits - 2, -1, -2):  # Moving backwards by 2 steps
        circuit.swap(i, i + 1)

    # Next, apply the inverse of the magnetic field rotations
    magnetic_field_angle = margin_of_error
    while magnetic_field_angle != 3.14 / 2:
        for qubit_index in range(num_qubits):
            circuit.ry(-rotation_angle, qubit_index)  # Applying the external rotation
            circuit.rz(-magnetic_field_angle, qubit_index)  # Apply the current magnetic field
        magnetic_field_angle += decay_rate


    # Now, inverse of creating the superposition state
    for qubit_index in range(num_qubits):
        circuit.ry(-3.14 / 2, qubit_index)


    # Measurement to read the qubit states
    circuit.measure(list(range(num_qubits)), list(range(num_qubits)))
    simulator = Aer.get_backend('qasm_simulator')
    compiled_circuit = transpile(circuit, simulator)
    result = execute(compiled_circuit, simulator, shots=shots).result()
    counts = result.get_counts()

    return binary_to_string(max(counts, key=counts.get).replace(' ', ''))

# Pygame initialization
pygame.init()
WIDTH, HEIGHT = 300, 200
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Quantum Decoded Character')
clock = pygame.time.Clock()
font = pygame.font.Font(None, 74)

decoded_string = ""  # Start with an empty string

running = True

text = font.render(decoded_string, True, BLACK)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:  # A key is pressed
            if event.unicode and len(event.unicode) == 1:  # If it's a single character
                decoded_string = quantum_encode_decode(event.unicode)
                text = font.render(decoded_string, True, BLACK)

    screen.fill(WHITE)
    screen.blit(text, (WIDTH // 2 - text.get_width() // 2, HEIGHT // 2 - text.get_height() // 2))
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
