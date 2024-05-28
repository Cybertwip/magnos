from datetime import datetime
import math

# Function to calculate days to desired phase angle between Earth and Mars
def days_to_phase_angle(desired_phase_angle):
    # Current date
    today = datetime.utcnow()
    j2000 = datetime(2000, 1, 1, 12)  # J2000.0 epoch

    # Calculate days from J2000.0 to today
    days_since_j2000 = (today - j2000).days

    # Constants for Earth and Mars orbital periods in Earth days
    orbital_period_earth = 365.25
    orbital_period_mars = 686.97

    # Angular velocities in degrees per day
    angular_velocity_earth = 360 / orbital_period_earth
    angular_velocity_mars = 360 / orbital_period_mars

    # Initial angles from J2000.0
    initial_angle_earth = (days_since_j2000 * angular_velocity_earth) % 360
    initial_angle_mars = (days_since_j2000 * angular_velocity_mars) % 360

    # Initialize angles and days for the simulation
    angle_earth = initial_angle_earth
    angle_mars = initial_angle_mars
    days = 0

    # Simulation to find when the phase angle reaches the desired value
    while True:
        angle_earth += angular_velocity_earth
        angle_mars += angular_velocity_mars
        days += 1

        # Calculate current phase angle
        phase_angle = abs((angle_mars - angle_earth) % 360)
        if phase_angle > 180:
            phase_angle = 360 - phase_angle

        # Check if the phase angle is close to the desired value
        if abs(phase_angle - desired_phase_angle) < 0.05:  # Small threshold for accuracy
            break

    return days, phase_angle

# Desired phase angle of 8 degrees
desired_angle = 180
days_until, phase_angle_at_day = days_to_phase_angle(desired_angle)
print(f"Days until phase angle is approximately {desired_angle} degrees: {days_until}, Phase Angle: {phase_angle_at_day:.2f} degrees")
