#pragma once

class PIDController {
public:
	float kp, ki, kd;

private:
	float integral = 0.0f;
	float prev_error = 0.0f;
	bool autoTuning = false;
	bool relayState = false; // Used for relay feedback, true for high, false for low
	float relayHigh = 1.0;   // Set a high relay value
	float relayLow = 0.0;   // Set a low relay value
	double period = 0.0f;
	double prevSwitchTime = 0.0f;
	double oscillationStartTime = 0.0f;
	int oscillationsCount = 0;
	
	
public:
	// Constructor
	PIDController(float kp, float ki, float kd);
	
	void startAutoTuning(float relayHigh, float relayLow);
	bool calibrating();
	bool getRelayState();
	bool calibrate(float normalizedError, float error, float dt, long long currentTime);
	float compute(float error, float dt);
};
