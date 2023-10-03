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


// rocket
namespace {
float scale_radius = 0.0001;

float thrust_multiplier = 100;
float isp_multiplier = 1;

float earth_radius = 6371000;
float moon_radius = 173710;
float rocket_height = 42;
float rocket_radius = 4;
float moon_distance = 384400e3;

float thrust_kN = 845 * thrust_multiplier; // Thrust of Merlin 1D engine in kN
float total_propellant_mass = 523000; // in kg

// Define the mass of Earth in your game's scale
float earth_mass = 5.972e24; // Mass of Earth in kg
float moon_mass = 7.342e22; // Mass of Moon in kg

// Falcon 9 dimensions
float radius = 1.83;
float height = 55.7;

// Falcon 9 dry mass (without fuel)
float dry_mass = 26000; // in kg

// Calculate the volume of the Falcon 9
float falcon9_volume = M_PI * pow(radius, 2) * height;

// Calculate the inferred average density
float average_density = dry_mass / falcon9_volume;

float earth_radius_game = earth_radius * scale_radius;
float moon_distance_game = moon_distance * scale_radius;
float moon_radius_game = moon_radius * scale_radius;

float rocket_height_game = 200 * scale_radius;
float rocket_radius_game = 50 * scale_radius;

float earth_position[3] = {0, 0, 0};
float moon_position[3] = {0, moon_distance, 0};

// Define the real-world distances for the atmospheric layers
float troposphere_distance = 120000;
float stratosphere_distance = 500000;
float mesosphere_distance = 850000;
float thermosphere_distance = 600000;

// Convert the real-world distances (now in km) to the game's scale
float troposphere_distance_game = troposphere_distance * scale_radius;
float stratosphere_distance_game = stratosphere_distance * scale_radius;
float mesosphere_distance_game = mesosphere_distance * scale_radius;
float thermosphere_distance_game = thermosphere_distance * scale_radius;


uint8_t troposphere_color[4] = {0, 0, 255, 100}; // Blue with alpha
uint8_t stratosphere_color[4] = {150, 150, 255, 40};
uint8_t mesosphere_color[4] = {80, 80, 200, 30};
uint8_t thermosphere_color[4] = {40, 40, 150, 20};
uint8_t exosphere_color[4] = {0, 0, 0};

// Initial gimbal rotation angle (in degrees)
float gimbal_rotation_angle = 0.0;

// Calculate the radii of each layer based on the game's scale
float troposphere_radius = earth_radius + troposphere_distance;
float stratosphere_radius = earth_radius + stratosphere_distance;
float mesosphere_radius = earth_radius + mesosphere_distance;
float thermosphere_radius = earth_radius + thermosphere_distance;

// Define atmospheric drag constants
float air_density = 1.225; // kg/m^3 (standard air density at sea level)
float drag_coefficient = 0.47; // A typical drag coefficient for a streamlined object
float frontal_area = M_PI * pow(rocket_radius, 2); // Using math.pi for the calculation

bool is_rocket_state = true;
float transition_speed = 2.0; // Adjust the transition speed as needed

// Given values
float mixture_ratio = 2.56; // LOX:RP-1

float density_LOX = 1141; // in kg/m^3
float density_RP1 = 820; // in kg/m^3

// Calculate mass of LOX and RP-1
float mass_LOX = total_propellant_mass / (1 + 1/mixture_ratio);
float mass_RP1 = total_propellant_mass - mass_LOX;

// Calculate volume of LOX and RP-1
float volume_LOX = mass_LOX / density_LOX;
float volume_RP1 = mass_RP1 / density_RP1;

float total_rocket_mass = dry_mass + total_propellant_mass;

// Constants
float G = 6.674e-11;
float total_distance = 2000;
float r_initial = moon_radius + total_distance;
}
