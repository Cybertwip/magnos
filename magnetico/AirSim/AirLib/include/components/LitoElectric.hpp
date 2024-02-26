#pragma once

#include "components/entities/Node.hpp"

#include <iostream>
#include <random>
#include <cmath>

class LitoElectric : public msr::airlib::Node {
public:
	float iron_thickness_mm;
	float alternator_size_mm;
	float iron_density;
	float resistance_ohms;
	float velocity;
	
	LitoElectric(float iron_thickness_mm, float alternator_size_mm, float iron_density, float resistance_ohms);
	
	float applyLaser(float laser_frequency_hz, float delta_time_sec);
	
	float calculate_force(float displacement_mm, float laser_intensity);
	
	float generate_energy(float frequency_hz);
	
	float calculate_laser_intensity(float laser_frequency_hz);
};
 
