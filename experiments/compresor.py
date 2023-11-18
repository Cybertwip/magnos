from pydub import AudioSegment
import numpy as np
import matplotlib.pyplot as plt

def measure_wavelength(audio_path, duration=10000):
    # Load audio file
    audio = AudioSegment.from_wav(audio_path)

    # Extract the first 10 seconds
    audio_10s = audio[:duration]

    # Get audio data as numpy array
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

# Replace 'your_audio_file.wav' with the path to your audio file
audio_path = 'protoman.wav'

# Measure the wavelength of the first 10 seconds in femtometers
wavelength_femtometers = measure_wavelength(audio_path)

# Set the compression factor (adjust as needed)
compression_factor = 100

# Simulate wavelength compression
compressed_wavelength = compress_wavelength(wavelength_femtometers, compression_factor)

# Plot the original and compressed wavelengths
plt.plot([0, 1], [wavelength_femtometers, wavelength_femtometers], label='Original Wavelength', linestyle='dashed')
plt.plot([0, 1], [compressed_wavelength, compressed_wavelength], label=f"Compressed Wavelength (Factor={compression_factor})")

plt.title("Original and Compressed Wavelengths")
plt.xlabel("Position along Axis")
plt.ylabel("Wavelength (femtometers)")
plt.legend()
plt.show()

print(f"The estimated wavelength is approximately {wavelength_femtometers:.2f} femtometers.")
