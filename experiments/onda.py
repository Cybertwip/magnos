from pydub import AudioSegment
import numpy as np
import matplotlib.pyplot as plt
from qutip import *
from scipy.interpolate import interp1d

def measure_wavelength(audio, samples):
    # Calculate the Fast Fourier Transform (FFT)
    fft_result = np.fft.fft(samples)
    fft_freq = np.fft.fftfreq(len(samples), 1 / audio.frame_rate)

    # Find the peak frequency
    peak_freq = np.abs(fft_freq[np.argmax(np.abs(fft_result))])

    # Calculate the wavelength using the speed of sound (assuming air at room temperature)
    speed_of_sound = 343  # meters per second
    wavelength_meters = speed_of_sound / peak_freq

    # Convert to femtometers
    wavelength_femtometers = wavelength_meters * 1e15

    return wavelength_femtometers

def estimate_wavelength_from_emf(emf_m_per_s_squared, speed_of_light=3e8):
    # Assuming emf_m_per_s_squared is the magnitude of the electromagnetic force
    # and it's related to the wavelength

    # Estimate wavelength in meters
    wavelength_meters = speed_of_light / np.sqrt(np.abs(emf_m_per_s_squared) + 1e-10)

    # Convert to femtometers
    wavelength_femtometers = wavelength_meters * 1e15

    return wavelength_femtometers


def compress_wavelength(wavelength, compression_factor):
    # Calculate the compressed wavelength
    compressed_wavelength = wavelength / compression_factor

    return compressed_wavelength

def acoustic_lens_simulation_compressed_wavelength(audio, samples, radius, compression_factor=2, num_points=500):
    # Measure the wavelength of the audio
    wavelength_femtometers = measure_wavelength(audio, samples)

    # Create an array of x-coordinates for visualization
    original_x_values = np.linspace(-2 * radius, 2 * radius, num_points)

    # Calculate the original wavelength
    original_wavelength = np.full_like(original_x_values, wavelength_femtometers)

    # Downsample the x-values to compress the wavelength information
    compressed_x_values = original_x_values[::compression_factor]

    # Calculate the compressed wavelength
    compressed_wavelength = compress_wavelength(wavelength_femtometers, compression_factor)

    return original_x_values, original_wavelength, compressed_x_values, compressed_wavelength

def calculate_number_of_protons(radius, compressed_wavelength):
    # Assuming a cylindrical volume for the compressed wavelength region
    volume_compressed_region = np.pi * radius**2 * 2 * radius * (compressed_wavelength / 1e15)

    # Volume occupied by a single proton (using average proton radius)
    volume_proton = (4/3) * np.pi * (0.84*1e-15)**3  # Proton radius is approximately 0.84 femtometers
    # Calculate the number of protons that can fit in the compressed region
    num_protons = volume_compressed_region / volume_proton

    return int(num_protons)
    
# Parameters for the simulation
audio_path = 'protoman.wav'
radius = 1e-8 * 0.29 # Reduce the radius to create a compressed region for a single proton
compression_factor = 10  # Adjust as needed

frequency_ratio = 2

audio = AudioSegment.from_wav("protoman.wav")

# Extract the first 10 seconds
duration = 1000  # in milliseconds
audio = audio[:duration]

# Get audio data as a numpy array
samples = np.array(audio.get_array_of_samples())

# Run the compressed simulation for protons with the same time duration as pair production
total_time_protons = duration / 1000 * np.pi / 2

num_time_steps_protons = len(samples) * frequency_ratio
times_protons = np.linspace(0, total_time_protons, num_time_steps_protons)

# Constants
N = 2
n_photons = 0
a = destroy(N)
H_field = a.dag() * a
psi0 = fock(N, n_photons)
total_time = total_time_protons
num_time_steps = num_time_steps_protons
times = times_protons

# Iron-related properties
d = 1.0  # Example value for the dipole moment
mu_iron = 2.0  # Example value for the magnetic moment of iron in Bohr magnetons
c = 3e8  # Speed of light in meters per second

# Iron masses to simulate
iron_mass = 0.00001

# Lists to store results
max_speeds_left = []
max_speeds_right = []
time_values = []


# Counter variables
time_counter_1 = 0
time_counter_2 = 0
time_counter_3 = 0


# Variables for time counter and polarity swapping
half_pi_time_reached = False
polarity_1l = 1  # Initial polarity
polarity_1r = 1  # Initial polarity
polarity_factor_1l = -1
polarity_factor_1r = 1


polarity_2l = 1  # Initial polarity
polarity_2r = 1  # Initial polarity
polarity_factor_2l = -1
polarity_factor_2r = 1

polarity_3l = 1  # Initial polarity
polarity_3r = 1  # Initial polarity
polarity_factor_3l = -1
polarity_factor_3r = 1

gravity_turn = 0

emf_left = []  # Store energy for the left iron bar
emf_right = []  # Store energy for the right iron bar

g = 9.81  # Acceleration due to gravity (m/s^2)

# Define time-dependent Hamiltonian
def hamiltonian_t(t, args):
    H_field = a.dag() * a
    H_left_iron = args['left_component_mass'] * (a.dag() + a)
    H_right_iron = args['right_component_mass'] * (a.dag() + a)
    return H_field + H_left_iron + H_right_iron

# Set up the arguments for the Hamiltonian
hamiltonian_args = {'left_component_mass': iron_mass, 'right_component_mass': iron_mass}

# Use sesolve to calculate the time-dependent results
result = sesolve(hamiltonian_t, psi0, tlist=times, e_ops=[a.dag() + a, a.dag() + a], args=(hamiltonian_args), options=Options(nsteps=10000))

# Calculate Casimir force values using a simple formula
emf_left_m_per_s_squared = (result.expect[0] * g)
emf_right_m_per_s_squared = (-result.expect[1] * g)

force_left_values = -np.pi / 2 * mu_iron * c / (240 * d**4) * np.gradient(np.real(emf_left_m_per_s_squared), times)

# Total time and number of time steps
simulation_time_factor = 9
total_time_pair_production = simulation_time_factor * 1e-15 * np.pi
times_pair_production = np.linspace(0, total_time_pair_production, num_time_steps)

index = 0
# Define the Hamiltonian for pair production with time-dependent EMF
def hamiltonian_pair_production_with_emf(t, args):
    global index
    omega = args['omega']  # Frequency of the incoming photon
    g = args['g']  # Coupling constant
    F_t = args['F_t']  # Time-dependent EMF function
    
    casimir_force = -np.pi / 2 * mu_iron * c / (240 * d**4) * F_t[index]
    index += 1

    # Hamiltonian terms
    H_photon = omega * a.dag() * a
    H_fermion = g * (a.dag() + a) * casimir_force
    return H_photon + H_fermion

# Set up the arguments for the Hamiltonian
hamiltonian_args_pair_production = {'omega': 2 * np.pi * 1e15, 'g': 1e-10, 'F_t': force_left_values}

# Initial state: vacuum state for the photon field
psi0_photon = fock(N, 0)

# Use sesolve to calculate the time-dependent results for pair production
result_pair_production_with_emf = sesolve(hamiltonian_pair_production_with_emf, psi0_photon,
                                           tlist=times_pair_production, args=(hamiltonian_args_pair_production),
                                           e_ops=[a.dag() * a, a.dag() + a],
                                           options=Options(nsteps=10000))

# Threshold for pair production
threshold = 1e-30
#pair_production_count = sum()
#print("Number of pair-produced particles:", pair_production_count)

force_left_values = -np.pi / 2 * mu_iron * c / (240 * d**4) * np.gradient(np.real(emf_left_m_per_s_squared), times)

energy_values = force_left_values**2 * hamiltonian_args['left_component_mass'] / 2

# Adjust power and frequency to very low values
power = 100  # Adjust the power to a very low value, in watts
frequency = 3000  # Adjust the frequency to a very low value, in hertz


# Calculate charge based on power and frequency
charge = power / frequency

# Convert energy values to volts
volts_values_left = np.array(energy_values) / charge


wavelength_values_femtometers = estimate_wavelength_from_emf(emf_left_m_per_s_squared)

# Run the compressed simulation for protons
original_x_values_protons, original_wavelength_protons, compressed_x_values_protons, compressed_wavelength_protons = acoustic_lens_simulation_compressed_wavelength(audio, samples, radius, compression_factor, num_points=num_time_steps_protons)
# Calculate the number of protons

num_protons = calculate_number_of_protons(radius, compressed_wavelength_protons)
print("Number of protons " + str(num_protons))
plt.figure(figsize=(14, 6))

plt.subplot(4, 1, 1)
plt.plot(times_pair_production, result_pair_production_with_emf.expect[0] > threshold, marker='o')
plt.xlabel('Time (s)')
plt.ylabel('Pairs produced')
plt.title('Pair Production: Particle Presence vs. Time')

# Plot volts values
plt.subplot(4, 1, 2)
plt.plot(times_pair_production, volts_values_left, label='3Khz photon laser')
plt.xlabel('Time (s)')
plt.ylabel('Energy (Volts)')
plt.legend()

# Plot wavelength
plt.subplot(4, 1, 3)
plt.plot(times_pair_production, wavelength_values_femtometers, label='Wavelength')
plt.xlabel('Time (s)')
plt.ylabel('Wavelength (femtometers)')
plt.legend()

plt.subplot(4, 1, 4)

plt.plot(original_x_values_protons, original_wavelength_protons, label='Original Wavelength', linestyle='dashed')
plt.plot(compressed_x_values_protons, np.full_like(compressed_x_values_protons, compressed_wavelength_protons), label=f"Compressed Wavelength (Factor={compression_factor})")

# Mark the position of a single proton within the compressed region

if num_protons > 0:
    plt.scatter([0], [compressed_wavelength_protons], color='red', label='Proton', marker='o')

plt.xlabel("Position along Lens (arbitrary units)")
plt.ylabel("Wavelength (femtometers)")
plt.legend()

# Ensure samples, original_x_values_protons, and compressed_x_values_protons have the same length

min_length = min(len(samples), len(original_x_values_protons), len(compressed_x_values_protons))
samples = samples[:min_length]
original_x_values_protons = original_x_values_protons[:min_length]
compressed_x_values_protons = compressed_x_values_protons[:min_length]

# Estimate the compressed audio waveform from the wavelength
compressed_audio_wave = np.interp(compressed_x_values_protons, original_x_values_protons, samples)
wavelength_femtometers = estimate_wavelength_from_emf(compressed_audio_wave)


# Create y-values for the pairs to match the structure
particle_presence = wavelength_values_femtometers > threshold

# Plot the combined waveform and proton position with a cross at the matching points
plt.figure(figsize=(14, 8))
plt.subplot(1, 1, 1)

# Create an interpolation function
interpolation_function = interp1d(original_x_values_protons, samples, kind='cubic', fill_value="extrapolate")

# Interpolate the compressed audio waveform from the wavelength
compressed_audio_wave = interpolation_function(compressed_x_values_protons)

y_time = len(compressed_audio_wave)

#y_time = 32

plt.plot(times_protons[0:y_time], emf_left_m_per_s_squared[0:y_time], label='Estimated EMF Wave', linestyle='dashed', color='green')
plt.plot(times_protons[0:y_time], compressed_audio_wave[0:y_time], label='Compressed Audio Wave', linestyle='dashed', color='blue')
plt.scatter(times_protons[0:y_time], particle_presence[0:y_time], color='cyan', label='Pairs', marker='^', s=100)  # Mark the position of a single proton

# Mark the position of protons within the compressed region
if num_protons > 0:
    proton_y_values = np.full_like(times_protons[0:y_time], num_protons)
    plt.scatter([times_protons[0:y_time]], proton_y_values, marker='o', color='red', label='Proton')

protons_per_second = (num_protons / total_time_protons) * len(times_protons[0:y_time])
print("Number of protons per second:", protons_per_second)

# Constants
avogadro_number = 6.022e23
molar_volume_stp = 22.4  # liters/mol
rate_of_generation = protons_per_second  # atoms per second

# Step 1: Number of moles of hydrogen gas needed to fill 1 liter at STP
volume_liters = 1
moles_of_hydrogen = volume_liters / molar_volume_stp

# Step 2: Total number of molecules
total_molecules = moles_of_hydrogen * avogadro_number

# Step 3: Time required
time_required_seconds = total_molecules / rate_of_generation

# Convert time to hours for better readability
time_required_hours = time_required_seconds / 3600

print("Time required to create 1 liter of hydrogen gas:")
print(f"In seconds: {time_required_seconds} seconds")
print(f"In hours: {time_required_hours:.2} hours")

tolerance = 1e32

matching_indices = np.where(np.abs(particle_presence[0:y_time] - compressed_audio_wave[0:y_time]) < tolerance)[0]

plt.scatter(times_protons[matching_indices], compressed_audio_wave[matching_indices], color='black', label='Matching particles', s=100)

#plt.ylim(0, len(compressed_audio_wave))
#plt.ylim(-compressed_wavelength_protons * 2, compressed_wavelength_protons * 2)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (femtometers)')
plt.title('Estimated Compressed Audio and EMF Waveforms over Time')
plt.ticklabel_format(style='plain', axis='x')  # Disable scientific notation on the x-axis

plt.legend()

# Constants
mass_of_positron = 9.10938356e-31  # kg (mass of positron)
charge_of_positron = 1.602176634e-19  # C (charge of positron)

# Simulation parameters
total_time = len(compressed_audio_wave)  # Total simulation time in seconds
time_values = np.linspace(0, total_time, num_time_steps)
dt = total_time / num_time_steps

# Arrays to store positron data
positions = np.zeros(num_time_steps)
velocities = np.zeros(num_time_steps)

# Initial conditions
positions[0] = 1e-3  # Initial position along the trap axis
velocities[0] = 1e4  # Initial velocity along the trap axis

# Lorentz force equation
def lorentz_force(q, v, emf):
    return q * emf

# Trap parameters
trap_center = 0  # Center of the trap
trap_width = 1e-3  # Width of the trap

# Simulation loop
for i in range(1, num_time_steps):
    t = time_values[i]

    # Get the electromagnetic force at the current time
    emf = emf_left_m_per_s_squared[i]

    # Lorentz force acting on the positron
    F = lorentz_force(charge_of_positron, np.array([0, velocities[i - 1], 0]), emf)

    # Update positron's velocity and position using Euler method
    velocities[i] = velocities[i - 1] + (F / mass_of_positron) * dt
    positions[i] = positions[i - 1] + velocities[i] * dt

    # Apply the trap - reflect positron if it goes beyond the trap boundaries
    if positions[i] < trap_center - trap_width / 2:
        positions[i] = trap_center - trap_width / 2
        velocities[i] = -velocities[i]  # Reflect the positron
    elif positions[i] > trap_center + trap_width / 2:
        positions[i] = trap_center + trap_width / 2
        velocities[i] = -velocities[i]  # Reflect the positron

# plt.subplot(3, 1, 2)

# # Plot positron's trajectory in the trap
# plt.plot(times_protons[:len(compressed_audio_wave)], positions[0:len(compressed_audio_wave)], label='Positron')
# plt.xlabel('Time')
# plt.ylabel('Position (m)')
# plt.title('Positron Trajectory in an EMF Trap')
# plt.legend()



# Function to calculate the Lorentz force on a charged particle
def lorentz_force(q, v, B, E):
    # q: charge, v: velocity vector, B: magnetic field vector, E: electric field vector
    return q * (np.cross(v, B) + E)

# Function to update the position and velocity of a charged particle using the Lorentz force
def update_particle_motion(q, m, x, v, dt, B, E):
    a = lorentz_force(q, v, B, E) / m  # acceleration
    v_new = v + a * dt
    x_new = x + v_new * dt
    return x_new, v_new

# Penning Trap Simulation


# Penning Trap Simulation
def penning_trap_simulation(q_particles, m_particles, x_particles, v_particles, dt, total_time, B, E_left, E_right):
    global num_time_steps
    num_particles = len(q_particles)
    time_values = np.linspace(0, total_time, num_time_steps)

    # Lists to store particle trajectories
    particle_positions = [[] for _ in range(num_particles)]

    for i in range(num_time_steps):
        for j in range(num_particles):
            # Use the Casimir force field as the electric field for the electron (negative charge)
            if q_particles[j] < 0:
                E = E_left[i]
            # Use emf_right_m_per_s_squared as the electric field for the proton (positive charge)
            else:
                E = E_right[i]

            x_particles[j], v_particles[j] = update_particle_motion(
                q_particles[j], m_particles[j], x_particles[j], v_particles[j], dt, B, E
            )
            particle_positions[j].append(x_particles[j].tolist())

    return time_values, particle_positions

# Constants
q_electron = -1.602176634e-19  # charge of an electron in coulombs
q_proton = 1.602176634e-19  # charge of a proton in coulombs
m_electron = 9.10938356e-31  # mass of an electron in kg
m_proton = 1.6726219e-27  # mass of a proton in kg

# Penning trap parameters
B_field = np.array([0, 0, 1e-5])  # static magnetic field along the z-axis
E_field = np.array([1e4, 0, 0])  # electric field along the x-axis

# Particle initialization
num_particles = 2
q_particles = [q_electron, q_proton]  # charges of electrons and protons
m_particles = [m_electron, m_proton]  # masses of electrons and protons

# Initial positions and velocities (for simplicity, placing particles on the x-axis)
x_particles = np.array([[1e-3, 1e-3, 0], [-1e-3, -1e-3, 0]])
v_particles = np.array([[0, -1e4, 0], [0, 1e4, 0]])

# Simulation parameters
dt = 1e-8  # time step
total_time = len(compressed_audio_wave)  # total simulation time

# Run Penning trap simulation
time_values, particle_positions = penning_trap_simulation(q_particles, m_particles, x_particles, v_particles, dt, total_time, B_field, emf_left_m_per_s_squared, emf_right_m_per_s_squared)

# plt.subplot(3, 1, 3)

# # Plot particle trajectories
# for i in range(num_particles):
#     positions = np.array(particle_positions[i])
#     plt.plot(times_protons[:len(compressed_audio_wave)], positions[0:len(compressed_audio_wave)], label=f'{["Electron", "Proton"][i]} Trajectory')

# plt.xlabel('Time (s)')
# plt.ylabel('Position along y-axis (m)')
# plt.title('Particle Trajectories in a Penning Trap')
# plt.legend()

plt.tight_layout()
plt.show()
