from pydub import AudioSegment
import numpy as np
import matplotlib.pyplot as plt

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

# Run the compressed simulation
original_x_values, original_wavelength, compressed_x_values, compressed_wavelength = acoustic_lens_simulation_compressed_wavelength(audio_path, radius, compression_factor)

# Calculate the number of protons
num_protons = calculate_number_of_protons(radius, compression_factor)

# Plot the original and compressed wavelengths
plt.plot(original_x_values, original_wavelength, label='Original Wavelength', linestyle='dashed')
plt.plot(compressed_x_values, np.full_like(compressed_x_values, compressed_wavelength), label=f"Compressed Wavelength (Factor={compression_factor})")

# Mark the position of a single proton within the compressed region
if num_protons > 0:
    plt.scatter([0], [compressed_wavelength], color='red', label='Proton', marker='o')

plt.title("Original and Compressed Wavelengths with Proton")
plt.xlabel("Position along Lens (arbitrary units)")
plt.ylabel("Wavelength (femtometers)")
plt.legend()
plt.show()

print(f"Number of protons within the compressed region: {int(num_protons)}")
