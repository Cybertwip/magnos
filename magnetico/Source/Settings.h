#pragma once
#include <cmath>

namespace{
float error_trial = 0.25f;
float global_delta = 16;
float fixed_delta = 1.0f / 60.0f;

int data_collection_bin_size = 4 /* mb */ * 1000 * 1024 ;

const float battery_voltage = 48;
const int number_of_gimbals = 6;
const int data_collection_mode_cycles = 2048 / number_of_gimbals;
const int min_voltage = 5;
const int max_voltage = 100 * number_of_gimbals / 3 * 2;
const int calibration_steps = 2; // 4 is optimal 2 is min vs time
const bool enable_lasers = true;
const int number_of_lasers = 6;
}

class Settings {
public:	
	static float desired_base_voltage;
	static float desired_target_voltage;
	static float desired_capacitor_voltage;
};


// Rocket parameters
namespace {
float rocket_height = 42;
float rocket_radius = 4;
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
float earth_radius_game = earth_radius;
float moon_distance_game = moon_distance;
float moon_radius_game = moon_radius;

// Atmospheric parameters
float troposphere_distance = 120000;
float stratosphere_distance = 500000;
float mesosphere_distance = 850000;
float thermosphere_distance = 600000;
float troposphere_distance_game = troposphere_distance;
float stratosphere_distance_game = stratosphere_distance;
float mesosphere_distance_game = mesosphere_distance;
float thermosphere_distance_game = thermosphere_distance;
float troposphere_radius = earth_radius + troposphere_distance;
float stratosphere_radius = earth_radius + stratosphere_distance;
float mesosphere_radius = earth_radius + mesosphere_distance;
float thermosphere_radius = earth_radius + thermosphere_distance;
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
float total_distance = 2000;
float r_initial = moon_radius + total_distance;
bool is_rocket_state = true;
float transition_speed = 2.0;
}
