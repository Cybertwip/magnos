#pragma once

namespace{
float error_trial = 0.0025f;
const int calibration_steps = 2;
float global_delta = 16;
float fixed_delta = 1.0f / 60.0f;
const bool adaptive_calibration = false;

const int data_collection_mode_cycles = 2400;
int data_collection_bin_size = 1 * 1000 * 1024; // 1 mb per collection

const float desired_base_voltage = 1.5;
const float adaptive_calibration_voltage = 3;
}

class Settings {
public:
	static bool data_collection_mode;
	static bool schedule_data_collection_mode;
	static int cycles_per_collection;
	static int desired_voltage_increase_per_second;
};
