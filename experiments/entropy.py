import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Function to calculate Shannon entropy of a signal
def calculate_entropy(signal):
    histogram, bin_edges = np.histogram(signal, bins=256, density=True)
    histogram = histogram[np.nonzero(histogram)]
    return -np.sum(histogram * np.log2(histogram))

# Create a noisy signal (random noise)
np.random.seed(0)
input_signal = np.random.normal(0, 1, 1000)

# Design a Butterworth low-pass filter
b, a = butter(N=3, Wn=0.1, btype='low')

# Arrays to store entropy values
input_entropies = []
output_entropies = []
negentropies = []

# Calculate entropies at different points in time
for i in range(100, len(input_signal)+1, 100):
    segment_input = input_signal[:i]
    segment_output = filtfilt(b, a, segment_input)
    
    input_entropy = calculate_entropy(segment_input)
    output_entropy = calculate_entropy(segment_output)
    negentropy = input_entropy - output_entropy
    
    input_entropies.append(input_entropy)
    output_entropies.append(output_entropy)
    negentropies.append(negentropy)

# Time points for plotting
time_points = range(100, len(input_signal)+1, 100)

# Plotting the entropies and negentropy over time
plt.figure(figsize=(12, 8))
plt.plot(time_points, input_entropies, label='Input Signal Entropy')
plt.plot(time_points, output_entropies, label='Output Signal Entropy')
plt.plot(time_points, negentropies, label='Negentropy (Decrease in Entropy)', linestyle='--')
plt.xlabel('Sample Size')
plt.ylabel('Entropy')
plt.title('Entropy and Negentropy Over Time')
plt.legend()
plt.grid(True)
plt.show()
