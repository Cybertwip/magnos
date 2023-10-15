#pragma once
#include <cmath>

namespace{
float global_delta = 16;
float fixed_delta = 1.0f / 60.0f;
const bool enable_lasers = true;
const int number_of_lasers = 20;
}

// Rocket parameters
namespace {
float scale_multiplier = 1;
float rocket_height = 49.5 * scale_multiplier;
float rocket_radius = 5.15 * scale_multiplier;
float thrust_kN = 845;
float total_propellant_mass = 523000;
float max_valve_opening = 1.0f;
float FALCON_9_BURN_RATE = 256.7f;

// Celestial body parameters
float earth_radius = 6371000;
float moon_radius = 173710;
float moon_distance = 384400e3;
float earth_mass = 5.972e24;
float moon_mass = 7.342e22;

// Atmospheric parameters
const float troposphere_distance = 12000.0f; // meters
const float stratosphere_distance = 50000.0f; // meters
const float mesosphere_distance = 80000.0f; // meters
const float thermosphere_distance = 600000.0f; // meters
const float troposphere_radius = earth_radius + troposphere_distance;
const float stratosphere_radius = earth_radius + stratosphere_distance;
const float mesosphere_radius = earth_radius + mesosphere_distance;
const float thermosphere_radius = earth_radius + thermosphere_distance;

uint8_t troposphere_color[4] = {0, 0, 255, 100};
uint8_t stratosphere_color[4] = {150, 150, 255, 40};
uint8_t mesosphere_color[4] = {80, 80, 200, 30};
uint8_t thermosphere_color[4] = {40, 40, 150, 20};
uint8_t exosphere_color[4] = {0, 0, 0};

// Rocket engine parameters
float mixture_ratio = 2.56;
float density_LOX = 1141;
float density_RP1 = 820;
float mass_LOX = total_propellant_mass / (1 + 1/mixture_ratio);
float mass_RP1 = total_propellant_mass - mass_LOX;
float volume_LOX = mass_LOX / density_LOX;
float volume_RP1 = mass_RP1 / density_RP1;
float dry_mass = 26000;
float total_rocket_mass = dry_mass + total_propellant_mass;
float gimbal_rotation_angle = 0.0;
float air_density = 1.225;
float drag_coefficient = 0.47;
float frontal_area = M_PI * pow(rocket_radius, 2);

// Simulation parameters
float G = 6.674e-11;
//float total_distance = 2000;
//float r_initial = moon_radius + total_distance;

float specific_impulse_at_sea_level = 311.0f; // seconds
float specific_impulse_in_vacuum = 282.0f; // seconds
float sea_level_pressure = 101325.0f; // Pascals
float vacuum_pressure = 0.0f; // Pascals
}

namespace{
void lerp(const uint8_t color1[4], const uint8_t color2[4], float ratio, uint8_t result[4]) {
	for (int i = 0; i < 4; i++) {
		result[i] = (1 - ratio) * color1[i] + ratio * color2[i];
	}
}
}
