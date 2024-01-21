
#include "components/LitoElectric.hpp"
#include "components/systems/CoilSystem.hpp"

LitoElectric::LitoElectric(float iron_thickness_mm, float alternator_size_mm, float iron_density, float resistance_ohms)
: iron_thickness_mm(iron_thickness_mm), alternator_size_mm(alternator_size_mm), iron_density(iron_density),
resistance_ohms(resistance_ohms), velocity(0) {
	
}

float LitoElectric::applyLaser(float laser_frequency_hz, float delta_time_sec) {
	float total_photon_energy = 0;
	float displacement_mm = (float)rand() / (float)RAND_MAX;
	float laser_intensity = calculate_laser_intensity(laser_frequency_hz);
	float force = calculate_force(displacement_mm, laser_intensity);
	float velocity_change = force / (0.5 * iron_density * iron_thickness_mm);
	velocity = velocity_change;
	
	float energy_generated = generate_energy(laser_frequency_hz);
	total_photon_energy += energy_generated;
	
	float current = total_photon_energy / delta_time_sec;
	float voltage = current * resistance_ohms;
	
	return voltage;
}

float LitoElectric::calculate_force(float displacement_nm, float laser_intensity) {
	// Example: Assuming laser_intensity is the intensity of the laser in watts per square meter
	float absorption_coefficient = 0.5f; // Example coefficient, adjust as needed
	
	// Convert displacement to meters
	float displacement_m = displacement_nm * 1e-9;
	
	// Calculate force based on laser intensity and displacement
	float force = laser_intensity * absorption_coefficient * iron_thickness_mm * displacement_m;
	
	return force;
}


float LitoElectric::generate_energy(float frequency_hz) {
	float energy_generated = 0.5 * iron_density * iron_thickness_mm * velocity * velocity;
	float photon_energy = energy_generated / frequency_hz;
	return photon_energy;
}

float LitoElectric::calculate_laser_intensity(float laser_frequency_hz) {
	// Example: Assuming a linear relationship between frequency and intensity
	float intensity_slope = 0.9f; // Adjust as needed
	float intensity_intercept = 1.0f; // Adjust as needed
	
	float intensity = intensity_slope * laser_frequency_hz + intensity_intercept;
	
	// Ensure intensity is non-negative
	intensity = std::max(0.0f, intensity);
	
	return intensity;
}
