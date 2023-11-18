import numpy as np
import matplotlib.pyplot as plt

def acoustic_lens_simulation_compressed_wavelength(radius, wavelength, compression_factor=2, num_points=500):
    """
    Simulate a compressed acoustic lens focusing sound waves.

    Parameters:
    - radius: Radius of the acoustic lens
    - wavelength: Wavelength of the sound wave
    - compression_factor: Factor by which to compress the wavelength information
    - num_points: Number of points for visualization

    Returns:
    - original_x_values: Original x-coordinates for the visualization
    - original_wavelength: Original wavelength at each x-coordinate
    - compressed_x_values: Compressed x-coordinates for the visualization
    - compressed_wavelength: Compressed wavelength at each x-coordinate
    """

    # Create an array of x-coordinates for visualization
    original_x_values = np.linspace(-2 * radius, 2 * radius, num_points)

    # Calculate the original wavelength
    original_wavelength = np.full_like(original_x_values, wavelength)

    # Downsample the x-values to compress the wavelength information
    compressed_x_values = original_x_values[::compression_factor]

    # Calculate the compressed wavelength in femtometers
    compressed_wavelength = np.full_like(compressed_x_values, wavelength * 1e15 / compression_factor)

    return original_x_values, original_wavelength, compressed_x_values, compressed_wavelength

def calculate_number_of_protons(radius, compression_factor):
    """
    Calculate the number of protons that can fit within the compressed wavelength region.

    Parameters:
    - radius: Radius of the compressed wavelength region (assumed spherical)
    - compression_factor: Factor by which the wavelength is compressed

    Returns:
    - num_protons: Number of protons that can fit within the region
    """

    # Assuming a spherical volume for the compressed wavelength region
    volume_compressed_region = (4/3) * np.pi * radius**3

    # Approximate volume occupied by a single proton (using average proton radius)
    volume_proton = (4/3) * np.pi * (0.84e-15)**3  # Proton radius is approximately 0.84 femtometers

    # Calculate the number of protons that can fit within the compressed wavelength region
    num_protons = volume_compressed_region / volume_proton

    return num_protons

# Parameters for the simulation
radius = 0.5  # Reduce the radius to create a compressed region for a single proton
wavelength = 0.1  # Assume the wavelength is in micrometers
compression_factor = 50  # Adjust as needed

# Run the compressed simulation
original_x_values, original_wavelength, compressed_x_values, compressed_wavelength = acoustic_lens_simulation_compressed_wavelength(radius, wavelength, compression_factor)

# Calculate the number of protons
num_protons = calculate_number_of_protons(radius, compression_factor)

# Plot the original and compressed wavelengths
plt.plot(original_x_values, original_wavelength, label='Original Wavelength', linestyle='dashed')
plt.plot(compressed_x_values, compressed_wavelength, label=f"Compressed Wavelength (Factor={compression_factor})")

# Mark the position of a single proton within the compressed region
plt.scatter([0], [wavelength * 1e15 / compression_factor], color='red', label='Proton', marker='o')

plt.title("Original and Compressed Wavelengths with Proton")
plt.xlabel("Position along Lens (arbitrary units)")
plt.ylabel("Wavelength (femtometers)")
plt.legend()
plt.show()

print(f"Number of protons within the compressed region: {int(num_protons)}")
