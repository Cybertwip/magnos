import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lfilter, butter
import pygame

# Photonic Oscillator
def photonic_oscillator(frequency, time, damping=0.01):
    amplitude = 1.0
    return amplitude * np.exp(-damping * time) * np.sin(2 * np.pi * frequency * time)

# Photodetector with noise
def photodetector(light_intensity):
    sensitivity = 0.8
    current = sensitivity * abs(light_intensity)
    noise = np.random.normal(0, 0.05, current.shape)
    return current + noise

# Lowpass filter
def lowpass_filter(signal, cutoff_freq, fs):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(1, normal_cutoff, btype='low', analog=False)
    y = lfilter(b, a, signal)
    return y

# Optical Repeater Boost
def optical_repeater_boost(intensity):
    boost_factor = 2.0
    return intensity * boost_factor

# Current to Voltage conversion
def current_to_voltage(current, resistance=1.0):
    return current * resistance

# Refractive index function for BK7
def n_BK7(lambda_microns):
    B1 = 1.03961212
    B2 = 0.231792344
    B3 = 1.01046945
    C1 = 0.00600069867
    C2 = 0.0200179144
    C3 = 103.560653
    
    n_squared = 1 + (B1 * lambda_microns**2) / (lambda_microns**2 - C1) + \
                (B2 * lambda_microns**2) / (lambda_microns**2 - C2) + \
                (B3 * lambda_microns**2) / (lambda_microns**2 - C3)
                
    return n_squared**0.5

# Parameters
frequency = 1.0
time = np.linspace(0, 10, 10000)
fs = len(time)/time[-1]  # Sampling frequency

# Simulate
light_intensity = photonic_oscillator(frequency, time)
current = photodetector(light_intensity)
filtered_current = lowpass_filter(current, 5.0, fs)
boosted_current = optical_repeater_boost(filtered_current)

voltage = current_to_voltage(filtered_current)
boosted_voltage = current_to_voltage(boosted_current)

# Initialize pygame
pygame.init()

# Colors and constants
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WIDTH, HEIGHT = 800, 600

# Screen and Clock
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Optical System Simulation")
clock = pygame.time.Clock()

# Font
font = pygame.font.SysFont(None, 36)

# For the visual effect
circle_radius = 0

# Main loop
accumulated_current = 0
accumulated_voltage = 0
running = True
simulate_effect = False
while running:
    screen.fill(WHITE)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                # Simulate
                light_intensity = photonic_oscillator(frequency, time)
                current = photodetector(light_intensity)
                filtered_current = lowpass_filter(current, 5.0, fs)
                boosted_current = optical_repeater_boost(filtered_current)

                voltage = current_to_voltage(filtered_current)
                boosted_voltage = current_to_voltage(boosted_current)

                # Accumulate values
                accumulated_current += np.mean(boosted_current)
                accumulated_voltage += np.mean(boosted_voltage)
                
                # Start the visual effect
                simulate_effect = True
                circle_radius = 0

    # Visual effect: Expanding circles representing light propagation
    if simulate_effect:
        if circle_radius > WIDTH:
            simulate_effect = False
        else:
            pygame.draw.circle(screen, BLUE, (WIDTH//2, HEIGHT//2), circle_radius, 2)
            circle_radius += 10

    # Display the accumulated values
    current_text = font.render(f"Accumulated Current: {accumulated_current:.2f}", True, (0, 0, 0))
    voltage_text = font.render(f"Accumulated Voltage: {accumulated_voltage:.2f}", True, (0, 0, 0))
    screen.blit(current_text, (50, 50))
    screen.blit(voltage_text, (50, 100))
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()