import matplotlib.pyplot as plt
import numpy as np

def magnetic_field(x, y, magnet_strength, magnet_distance):
    """
    Calculate the magnetic field at a given point (x, y) for two magnets.

    Parameters:
    - x, y: Coordinates of the point
    - magnet_strength: Strength of the magnets
    - magnet_distance: Distance between the magnets

    Returns:
    - Magnetic field vector (Bx, By)
    """
    # Distance from the top magnet
    d1 = np.sqrt((x)**2 + (y - magnet_distance / 2)**2)
    # Distance from the bottom magnet
    d2 = np.sqrt((x)**2 + (y + magnet_distance / 2)**2)

    # Magnetic field components for each magnet
    Bx1 = magnet_strength * (y - magnet_distance / 2) / (d1**3)
    By1 = magnet_strength * x / (d1**3)

    Bx2 = magnet_strength * (y + magnet_distance / 2) / (d2**3)
    By2 = magnet_strength * x / (d2**3)

    # Total magnetic field
    Bx = Bx1 + Bx2
    By = By1 + By2

    return Bx, By

def simulate_copper_layer(magnet_strength=1, magnet_distance=2, grid_size=100, time_steps=100, dt=0.1):
    """
    Simulate the movement of a copper layer between two magnets.

    Parameters:
    - magnet_strength: Strength of the magnets
    - magnet_distance: Distance between the magnets
    - grid_size: Number of points in each dimension for the grid
    - time_steps: Number of time steps
    - dt: Time step size

    Returns:
    - Visualization of the movement of the copper layer
    """
    # Create a grid of points
    x = np.linspace(-5, 5, grid_size)
    y = np.linspace(-5, 5, grid_size)
    X, Y = np.meshgrid(x, y)

    # Initial position of the copper layer
    layer_position = np.zeros_like(X)

    # Plot the initial position
    plt.figure(figsize=(6, 6))
    plt.pcolor(X, Y, layer_position, cmap='viridis')
    plt.colorbar(label='Copper Layer Position')
    plt.title('Copper Layer Initial Position')
    plt.show()

    # Simulation loop
    for step in range(time_steps):
        # Calculate the magnetic field at each point
        Bx, By = magnetic_field(X, Y, magnet_strength, magnet_distance)

        # Update the position of the copper layer based on the induced current
        current = -Bx  # Assuming a simple relation between magnetic field and current density
        layer_position += current * dt

        # Plot the updated position at regular intervals
        if step % 10 == 0:
            plt.figure(figsize=(6, 6))
            plt.pcolor(X, Y, layer_position, cmap='viridis')
            plt.colorbar(label='Copper Layer Position')
            plt.title(f'Copper Layer Position - Step {step}')
            plt.show()

# Run the simulation
simulate_copper_layer()
