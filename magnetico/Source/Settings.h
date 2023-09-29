#pragma once

namespace{
float error_trial = 0.25f;
const int calibration_steps = 4;
float global_delta = 16;
float fixed_delta = 1.0f / 60.0f;

const int data_collection_mode_cycles = 2048;
int data_collection_bin_size = 4 /* mb */ * 1000 * 1024 ;

const int min_voltage = 5;
const int max_voltage = 12;

const float adaptive_calibration_voltage = 3;
}

class Settings {
public:
	static bool data_collection_mode;
	static bool schedule_data_collection_mode;
	static bool schedule_recalibration_for_collection;
	
	static int cycles_per_collection;
	static float desired_base_voltage;
	static float desired_target_voltage;
};
