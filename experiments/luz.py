import numpy as np
import matplotlib.pyplot as plt

def refract(point, center, n1, n2, wavelength):
    d = np.linalg.norm(point - center)
    angle = np.arctan2(point[1] - center[1], point[0] - center[0])
    new_angle = angle + np.arcsin((n1 / n2) * np.sin(angle))
    new_point = center + d * np.array([np.cos(new_angle), np.sin(new_angle)])
    new_wavelength = wavelength * (n1 / n2)
    return new_point, new_wavelength

def simulate_refraction(num_lenses, lens_radius, n_air, n_lens, initial_wavelength):
    center = np.array([0, 0])
    points = [np.array([1, 0])]
    wavelengths = [initial_wavelength]

    for _ in range(num_lenses):
        new_point, new_wavelength = refract(points[-1], center, n_air, n_lens, wavelengths[-1])
        points.append(new_point)
        wavelengths.append(new_wavelength)

        angle = np.arctan2(new_point[1] - center[1], new_point[0] - center[0])
        center[0] += 2 * lens_radius * np.cos(angle)
        center[1] += 2 * lens_radius * np.sin(angle)

    return np.array(points), np.array(wavelengths)

def plot_simulation(path, wavelengths, lens_radius, title):
    plt.subplot(2, 1, 1)

    plt.plot(path[:, 0], path[:, 1], marker='o', label='Light Path')
    plt.scatter([0], [0], color='red', marker='o', label='Lens Centers')
    for i in range(len(path)):
        circle = plt.Circle((path[i, 0], path[i, 1]), lens_radius, color='blue', fill=False)
        plt.gca().add_patch(circle)

    plt.xlabel('Position along Light Path (X-axis)')
    plt.ylabel('Position Perpendicular to Light Path (Y-axis)', color='tab:blue')
    plt.tick_params(axis='y', labelcolor='tab:blue')
    #plt.set_title(title)
    plt.legend(loc='upper left')


    plt.subplot(2, 1, 2)

    plt.plot(range(len(wavelengths)), wavelengths, color='tab:green', linestyle='dashed', marker='o', label='Wavelength')
    plt.xlabel('Time Step')
    plt.ylabel('Wavelength (nm)', color='tab:green')
    plt.tick_params(axis='y', labelcolor='tab:green')
    plt.legend(loc='upper right')

    plt.grid(True)
    plt.tight_layout()
    plt.show()    

# Parameters
num_lenses = 10
lens_radius = 1.0
n_air = 1.0
n_lens = 1.5
initial_wavelength = 500.0  # Initial wavelength in nanometers

# Simulate and plot the path of light and wavelength changes
light_path, wavelength_changes = simulate_refraction(num_lenses, lens_radius, n_air, n_lens, initial_wavelength)
plot_simulation(light_path, wavelength_changes, lens_radius, 'Simulation of Light Refraction through Circular Lenses')
