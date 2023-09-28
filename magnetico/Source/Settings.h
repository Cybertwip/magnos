#pragma once

namespace{
float error_trial = 0.0025f;
const float global_timestep = 60.0f;
const int calibration_steps = 8;
const int calibration_time = 1000;
float global_delta = 1.0f / global_timestep;
const bool adaptive_calibration = false;

const int data_collection_mode_cycles = 2400;
int data_collection_bin_size = 1024 * 1000 * 24; // 5 mb

const float desired_base_voltage = 1.5;
const float adaptive_calibration_voltage = 3;
}

class Settings {
public:
	static bool data_collection_mode;
	static int cycles_per_collection;
	static int desired_voltage_increase_per_second;
};
