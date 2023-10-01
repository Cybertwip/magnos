#include "PIDController.h"

#include "Settings.h"

// Constructor
PIDController::PIDController(float kp, float ki, float kd)
: kp(kp), ki(ki), kd(kd) {
}

void PIDController::startAutoTuning(float relayHigh, float relayLow) {
	autoTuning = true;
	relayState = false;
	relayState = !relayState;
	this->relayHigh = relayHigh;
	this->relayLow = relayLow;
	oscillationsCount = 0;
	period = 0;
	prevSwitchTime = 0;
	prev_error = 0;
}

bool PIDController::calibrating() {
	return autoTuning;
}

bool PIDController::getRelayState() {
	return relayState;
}

bool PIDController::calibrate(float normalizedError, float error, float dt, long long currentTime){
	if (autoTuning) {
		if ((relayState && normalizedError > 0.5f) || (!relayState && normalizedError < 0.5f)) {
			// Capture the oscillation period
			if (oscillationsCount > 0) {
				period += currentTime - prevSwitchTime;
			} else if (oscillationsCount == 0) {
				oscillationStartTime = currentTime;
			}
			oscillationsCount++;
			prevSwitchTime = currentTime;
			
			relayState = !relayState;
			
			if (oscillationsCount >= calibration_steps) {  // consider 5 oscillations for averaging
				autoTuning = false;
				period /= oscillationsCount - 1;  // Average period
				// Ziegler-Nichols method
				kp = 0.6 * relayHigh / period;
				ki = 2 * kp / period;
				kd = kp * period / 8;
			}
		}
	}
	
	auto _ = compute(error, dt);
	
	return autoTuning;
	
}

float PIDController::compute(float error, float dt) {

	// Proportional term
	float p = kp * error;

	// Integral term with consideration of dt
	integral += error * dt;

	// Derivative term and consider dt
	float d = kd * (error - prev_error) / dt;

	// Compute the output
	float output = p + ki * integral + d;

	// Save the current error for the next iteration
	prev_error = error;

	return output;
}
