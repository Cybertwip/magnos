import numpy as np
import matplotlib.pyplot as plt

# Parameters
num_levels = 5
initial_radius = 5
ring_thickness = 2
spacing = 2
mu_0 = 4 * np.pi * 1e-7  # Magnetic constant (permeability of free space)
material_permeability = 1.2e-6  # Permeability of the material
applied_voltage = 220  # Volts
resistance = 1  # Ohms (Assumed for simplicity)
mass_of_ball = 0.1  # kg (Assumed mass of the ball)
distance_over_which_force_acts = 1  # meters (Assumed distance)
time_period = 2  # seconds, assumed duration over which the acceleration occurs

# Calculate current from applied voltage
current_from_voltage = applied_voltage / resistance  # Ohms Law

# Function to calculate magnetic field and magnetic induction due to a current loop
def magnetic_fields(radius, current, position):
    r = np.linalg.norm(position[:2])  # Distance from the center of the loop
    theta = np.arctan2(position[1], position[0])  # Angle for the dl calculation
    dl = np.array([-radius * np.sin(theta), radius * np.cos(theta), 0])  # Infinitesimal element of current loop
    cross_product = np.cross(dl, position - np.array([0, 0, position[2]]))
    magnetic_field = (mu_0 * current / (4 * np.pi)) * cross_product / (r**3)

    # Adjust magnetic induction (B-field) calculation to include material permeability
    magnetic_induction = material_permeability * (current / (2 * np.pi * r)) * np.array([np.sin(theta), -np.cos(theta), 0])

    return magnetic_field, magnetic_induction

# Lists to store magnetic pull for each ring and the total magnetic pull
magnetic_pull_per_ring = []
total_magnetic_pull = 0

# Loop to calculate magnetic pull for each level
for level in range(num_levels):
    current_radius = initial_radius + level * spacing
    current_thickness = ring_thickness + level

    magnetic_pull_for_level = 0

    # Loop through the ball crossing the ring
    for theta in np.linspace(0, 2 * np.pi, 100):
        ball_position = np.array([current_radius * np.cos(theta), current_radius * np.sin(theta), level * (ring_thickness + spacing)])
        
        # Enhanced current due to applied voltage
        current = current_from_voltage * 2 * np.pi * current_radius * current_thickness  # Total current in the loop, adjusted by applied voltage

        # Calculate magnetic field and magnetic induction
        magnetic_field, magnetic_induction = magnetic_fields(current_radius, current, ball_position)

        # Calculate magnetic pull using the induced B-field
        magnetic_pull = np.linalg.norm(magnetic_induction)
        magnetic_pull_for_level += magnetic_pull

    magnetic_pull_per_ring.append(magnetic_pull_for_level)
    total_magnetic_pull += magnetic_pull_for_level

# Calculate work done by the total magnetic pull over the assumed distance
work_done = total_magnetic_pull * distance_over_which_force_acts

# Calculate final speed of the ball using the work-energy principle
final_speed = np.sqrt(2 * work_done / mass_of_ball)

# Speed Evolution
# Calculate acceleration
acceleration = total_magnetic_pull / mass_of_ball  # a = F/m

# Time intervals for plotting speed evolution
time_intervals = np.linspace(0, time_period, 100)

# Calculate speed at each time interval assuming constant acceleration (v = at)
speed_evolution = acceleration * time_intervals

# Parameters for subplot layout
nrows = 2
ncols = 2

fig, axs = plt.subplots(nrows, ncols, figsize=(12, 10))
fig.suptitle('Magnetic System Analysis')

# Magnetic Pull for Each Ring
axs[0, 0].plot(np.arange(num_levels), magnetic_pull_per_ring, marker='o', linestyle='-', color='blue')
axs[0, 0].set_title('Magnetic Pull for Each Ring')
axs[0, 0].set_xlabel('Ring Level')
axs[0, 0].set_ylabel('Magnetic Pull (N)')
axs[0, 0].grid(True)

# Total Magnetic Pull and Final Speed of the Ball
categories = ['Total Magnetic Pull (N)', 'Final Speed of the Ball (m/s)']
values = [total_magnetic_pull, final_speed]
axs[0, 1].bar(categories, values, color=['blue', 'green'])
axs[0, 1].set_title('Total Magnetic Pull & Final Speed')
for index, value in enumerate(values):
    axs[0, 1].text(index, value, f'{value:.2f}', ha='center', va='bottom')
axs[0, 1].set_ylabel('Values')

# Speed Evolution with Applied Voltage Annotation
time_intervals = np.linspace(0, time_period, 100)  # Assuming time_period is defined
speed_evolution = acceleration * time_intervals  # Assuming acceleration is calculated
axs[1, 0].plot(time_intervals, speed_evolution, label='Speed Evolution', color='red')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Speed (m/s)')
axs[1, 0].set_title('                               Speed')
axs[1, 0].annotate(f'Applied Voltage: {applied_voltage}V', xy=(0.5, 0.9), xycoords='axes fraction', ha='center', va='center', fontsize=9, bbox=dict(boxstyle="round,pad=0.3", edgecolor="black", facecolor="white"))
axs[1, 0].grid(True)
axs[1, 0].legend()

# Hide the 4th plot (bottom right) as it's unused
axs[1, 1].axis('off')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust the layout to make room for the main title
plt.show()
