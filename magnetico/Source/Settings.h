#pragma once

namespace {
float error_trial = 0.0025f;
const float global_timestep = 60.0f;
const int calibration_steps = 8;
const int calibration_time = 1.0f;
float global_delta = 1.0f / global_timestep;
const bool adaptive_calibration = false;
const bool data_collection_mode = false;

const int cycles_per_collection = data_collection_mode ? 2000 : 1;

const float desired_base_voltage = 1.5;
float desired_voltage_increase_per_second = 7;
const float adaptive_calibration_voltage = 3;
}
