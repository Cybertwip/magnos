#pragma once

#include "components/entities/Node.hpp"

class Laser : public Node {
private:
	double apertureRadius;   // Aperture radius in meters
	bool isConvexLens;       // Whether the lens is convex (true) or concave (false)
	double focalLength;      // Focal length of the lens in meters
	double voltageInput;     // Voltage input in volts
	
public:
	// Constructor to initialize the Laser object
	Laser(float apertureRadius, float isConvexLens, float focalLength, float voltageInput, float frequency);
	
	// Set the aperture radius
	void setApertureRadius(float radius);
	
	// Get the aperture radius
	float getApertureRadius() const;
	
	// Set the lens type (convex or concave)
	void setLensType(bool convex);
	
	// Check if the lens is convex
	bool isConvex() const;
	
	// Set the focal length of the lens
	void setFocalLength(float length);
	
	// Get the focal length of the lens
	float getFocalLength() const;
	
	// Set the voltage input
	void setVoltageInput(float voltage);
	
	// Get the voltage input
	float getVoltageInput() const;
	
	// Calculate and print the laser power
	void calculateLaserPower() const;
	
	void update(float dt) override;
	
	void simulateOpticalSystem(float dt);
	
	float getAccumulatedVoltage() const;
	void dischargeAccumulatedVoltage(float dischargeAmount);
	
	float getGuiMeasure() const;

private:
	
	float accumulatedVoltage;
	float totalTime;  // Added member to keep track of total time
	float timeElapsed;  // Added member to keep track of time elapsed
	float frequency;       // Added member for the laser's frequency
	
	float maxAccumulatedVoltage = 20;
	
	float voltagePerSecond = 0.0f;
	float guiMeasure = 0.0f;
	
	std::vector<float> currentSamples;

};
