from pydub import AudioSegment
import numpy as np
import matplotlib.pyplot as plt
from qutip import *
from scipy.interpolate import interp1d

def measure_wavelength(audio_path, duration=10000):
    # Load audio file
    audio = AudioSegment.from_wav(audio_path)

    # Extract the first 10 seconds
    audio_10s = audio[:duration]

    # Get audio data as a numpy array
    samples = np.array(audio_10s.get_array_of_samples())

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

def acoustic_lens_simulation_compressed_wavelength(audio_path, radius, compression_factor=2, num_points=500):
    # Measure the wavelength of the audio
    wavelength_femtometers = measure_wavelength(audio_path)

    # Create an array of x-coordinates for visualization
    original_x_values = np.linspace(-2 * radius, 2 * radius, num_points)

    # Calculate the original wavelength
    original_wavelength = np.full_like(original_x_values, wavelength_femtometers)

    # Downsample the x-values to compress the wavelength information
    compressed_x_values = original_x_values[::compression_factor]

    # Calculate the compressed wavelength
    compressed_wavelength = compress_wavelength(wavelength_femtometers, compression_factor)

    return original_x_values, original_wavelength, compressed_x_values, compressed_wavelength

def calculate_number_of_protons(radius, compression_factor):
    # Assuming a cylindrical volume for the compressed wavelength region
    volume_compressed_region = np.pi * radius**2 * 2 * radius

    # Volume occupied by a single proton (using average proton radius)
    volume_proton = (4/3) * np.pi * (0.84e-15)**3  # Proton radius is approximately 0.84 femtometers

    # Check if the compressed region can contain at least one proton
    if volume_compressed_region >= volume_proton:
        return 1
    else:
        return 0

# Parameters for the simulation
audio_path = 'protoman.wav'
radius = 0.5  # Reduce the radius to create a compressed region for a single proton
compression_factor = 50  # Adjust as needed




# Constants
N = 2
n_photons = 0
a = destroy(N)
H_field = a.dag() * a
psi0 = fock(N, n_photons)
total_time = np.pi 
num_time_steps = 10
times = np.linspace(0, total_time, num_time_steps)

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
total_time_pair_production = 1e-15 * np.pi 
num_time_steps_pair_production = 10
times_pair_production = np.linspace(0, total_time_pair_production, num_time_steps_pair_production)

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

audio = AudioSegment.from_wav("protoman.wav")
# Extract the first 10 seconds
duration = 10000  # in milliseconds
audio_10s = audio[:duration]

# Get audio data as a numpy array
samples = np.array(audio_10s.get_array_of_samples())

# Create time values for x-axis
time_values = np.linspace(0, duration / 1000, len(samples))

# Run the compressed simulation for protons with the same time duration as pair production
total_time_protons = duration / 1000
num_time_steps_protons = len(samples)
times_protons = np.linspace(0, total_time_protons, num_time_steps_protons)

# Run the compressed simulation for protons
original_x_values_protons, original_wavelength_protons, compressed_x_values_protons, compressed_wavelength_protons = acoustic_lens_simulation_compressed_wavelength(audio_path, radius, compression_factor, num_points=num_time_steps_protons)
# Calculate the number of protons
num_protons = calculate_number_of_protons(radius, compression_factor)
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

plt.title("Original and Compressed Wavelengths with Proton")
plt.xlabel("Position along Lens (arbitrary units)")
plt.ylabel("Wavelength (femtometers)")
plt.legend()

# Ensure samples, original_x_values_protons, and compressed_x_values_protons have the same length
min_length = min(len(samples), len(times_pair_production), len(compressed_x_values_protons))
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
plt.subplot(2, 1, 1)

# Create an interpolation function
interpolation_function = interp1d(original_x_values_protons, samples, kind='cubic', fill_value="extrapolate")

# Interpolate the compressed audio waveform from the wavelength
compressed_audio_wave = interpolation_function(compressed_x_values_protons)

plt.plot(times_protons[:len(compressed_audio_wave)], emf_left_m_per_s_squared, label='Estimated EMF Wave', linestyle='dashed', color='green')
plt.plot(times_protons[:len(compressed_audio_wave)], compressed_audio_wave, label='Compressed Audio Wave', linestyle='dashed', color='blue')
plt.scatter(times_protons[:len(compressed_audio_wave)], particle_presence, color='cyan', label='Pairs', marker='^', s=100)  # Mark the position of a single proton

if num_protons > 0:
    plt.scatter([times_protons[0:len(compressed_audio_wave)]], [compressed_audio_wave[0:len(compressed_audio_wave)]], color='red', label='Proton', marker='o')

tolerance = 10000000

matching_indices = np.where(np.abs(particle_presence - compressed_audio_wave) < tolerance)[0]

plt.scatter(times_protons[matching_indices], compressed_audio_wave[matching_indices], marker='x', color='black', label='Matching particles', s=100)
plt.ylim(0, len(compressed_audio_wave))
plt.ylim(-compressed_wavelength_protons * 2, compressed_wavelength_protons * 2)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (femtometers)')
plt.title('Estimated Compressed Audio and EMF Waveforms over Time')
plt.legend()

# Constants
mass_of_positron = 9.10938356e-31  # kg (mass of positron)
charge_of_positron = 1.602176634e-19  # C (charge of positron)

# Simulation parameters
total_time = len(compressed_audio_wave)  # Total simulation time in seconds
num_time_steps = 10  # Number of time steps
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

plt.subplot(2, 1, 2)

# Plot positron's trajectory in the trap
plt.plot(times_protons[:len(compressed_audio_wave)], positions, label='Positron')
plt.xlabel('Time')
plt.ylabel('Position (m)')
plt.title('Positron Trajectory in an EMF Trap')
plt.legend()

plt.tight_layout()
plt.show()
