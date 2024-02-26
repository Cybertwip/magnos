
#include "components/Laser.hpp"
#include "components/systems/CoilSystem.hpp"


// Photonic Oscillator
// Constants specific to the photodetector
const float SENSITIVITY = 0.8f;   // Sensitivity factor (adjust as needed)
const float DARK_CURRENT = 0.03f;   // Dark current (Amps, typically very small)
const float LOAD_RESISTANCE = 6.0f; // Load resistor value (Ohms, adjust as needed)

// Function to convert light frequency to voltage
float convertLightToVoltage(float frequency, float delta_time) {
	// Calculate the optical power (Watts) using the frequency and sensitivity
	float optical_power = frequency * SENSITIVITY;
	
	// Calculate the photocurrent (Amps) using the optical power and dark current
	float photocurrent = (optical_power + DARK_CURRENT) * delta_time;
	
	// Calculate the voltage across the load resistor
	float voltage = photocurrent * LOAD_RESISTANCE;
	
	return voltage;
}

// Photodetector with noise
float photodetector(float lightIntensity) {
	float sensitivity = 0.8f;
	float current = sensitivity * std::fabs(lightIntensity);
	std::random_device rd;
	std::default_random_engine gen(rd());
	std::normal_distribution<float> noise(0, 0.05f);
	return current + noise(gen);
}

// Lowpass filter
std::vector<float> lowpassFilter(const std::vector<float>& signal, float cutoffFreq, float fs) {
	float nyquist = 0.5f * fs;
	float normalCutoff = cutoffFreq / nyquist;
	float omegaC = 2.0f * M_PI * normalCutoff;
	float alpha = std::sin(omegaC) / (2.0f * 0.7071f); // 0.7071 is the Butterworth filter's damping ratio
	std::vector<float> b = { alpha, alpha };
	std::vector<float> a = { 1.0f + alpha, -1.0f };
	std::vector<float> y(signal.size(), 0.0f);
	
	// Initialize the first two samples
	y[0] = 0.0f;
	y[1] = 0.0f;
	
	for (size_t i = 2; i < signal.size(); ++i) {
		y[i] = (b[0] * signal[i] + b[1] * signal[i - 1] - a[1] * y[i - 1]) / a[0];
	}
	
	return y;
}

// Optical Repeater Boost
float opticalRepeaterBoost(float intensity) {
	float boostFactor = 2.0f;
	return intensity * boostFactor;
}

// Current to Voltage conversion
float currentToVoltage(float current, float resistance = 4.0f) {
	return current * resistance;
}

// Constructor to initialize the Laser object
Laser::Laser(float apertureRadius, float isConvexLens, float focalLength, float voltageInput, float frequency) {
	this->apertureRadius = apertureRadius;
	this->isConvexLens = isConvexLens;
	this->focalLength = focalLength;
	this->voltageInput = voltageInput;
	this->frequency = frequency;
}

// Set the aperture radius
void Laser::setApertureRadius(float radius) {
	apertureRadius = radius;
}

// Get the aperture radius
float Laser::getApertureRadius() const {
	return apertureRadius;
}

// Set the lens type (convex or concave)
void Laser::setLensType(bool convex) {
	isConvexLens = convex;
}

// Check if the lens is convex
bool Laser::isConvex() const {
	return isConvexLens;
}

// Set the focal length of the lens
void Laser::setFocalLength(float length) {
	focalLength = length;
}

// Get the focal length of the lens
float Laser::getFocalLength() const {
	return focalLength;
}

float Laser::getFrequency() const {
	return frequency;
}

// Set the voltage input
void Laser::setVoltageInput(float voltage) {
	if(voltage == 0){
		voltageInput = 0;
	} else {
		voltageInput += voltage;
		
		if(voltageInput >= Settings::desired_laser_voltage){
			voltageInput = Settings::desired_laser_voltage;
		}
	}
}

// Get the voltage input
float Laser::getVoltageInput() const {
	return voltageInput;
}

// Calculate and print the laser power
void Laser::calculateLaserPower() const {
	// Simplified power calculation (not accounting for real-world optics)
}
void Laser::update(float dt) {
	// Update the laser light's color based on the amplified voltage
	//updateLaserLightColor();
	
	// Simulate the optical system and accumulate values based on delta time (dt)
	
	if(getVoltageInput() >= Settings::desired_laser_voltage){
		simulateOpticalSystem(dt);
	}
	
	// Accumulate time elapsed
	timeElapsed += dt;
	
	// Check if it's time to reset the simulation
	if (timeElapsed >= totalTime) {
		timeElapsed = 0.0f;
		
		guiMeasure = voltagePerSecond;
		
		voltagePerSecond = 0;
	}
}

void Laser::simulateOpticalSystem(float dt) {
 	// Apply photodetector
	float currentSample = convertLightToVoltage(frequency, dt);
	
	// Accumulate values based on delta time (dt)
	accumulatedVoltage += currentSample;
	voltagePerSecond += currentSample;
	
	accumulatedVoltage = std::min(maxAccumulatedVoltage, accumulatedVoltage);
	
	voltagePerSecond = std::min(maxAccumulatedVoltage, voltagePerSecond);
	
}

float Laser::getAccumulatedVoltage() const {
	return accumulatedVoltage;
}

void Laser::dischargeAccumulatedVoltage(float dischargeAmount) {
	accumulatedVoltage -= dischargeAmount;
	
	// Ensure accumulatedVoltage doesn't go below zero
	if (accumulatedVoltage < 0) {
		accumulatedVoltage = 0;
	}
}

float Laser::getGuiMeasure() const {
	return guiMeasure;
}
