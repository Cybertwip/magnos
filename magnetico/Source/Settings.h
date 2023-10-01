#pragma once

namespace{
float error_trial = 0.25f;
float global_delta = 16;
float fixed_delta = 1.0f / 60.0f;

int data_collection_bin_size = 4 /* mb */ * 1000 * 1024 ;

const float battery_voltage = 12;
const int number_of_gimbals = 6;
const int data_collection_mode_cycles = 2048 / number_of_gimbals;
const int min_voltage = 5;
const int max_voltage = 10 * number_of_gimbals / 3 * 2;
const int calibration_steps = 2; // 4 is optimal 2 is min vs time
const bool enable_lasers = true;
}

class Settings {
public:	
	static float desired_base_voltage;
	static float desired_target_voltage;
	static float desired_capacitor_voltage;
};
