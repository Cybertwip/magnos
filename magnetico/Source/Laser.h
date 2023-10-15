#pragma once

#include <axmol.h>
#include "extensions/Particle3D/PU/PUParticleSystem3D.h"


class Laser : public ax::Node {
private:
	double apertureRadius;   // Aperture radius in meters
	bool isConvexLens;       // Whether the lens is convex (true) or concave (false)
	double focalLength;      // Focal length of the lens in meters
	double voltageInput;     // Voltage input in volts
	
public:
	// Constructor to initialize the Laser object
	Laser(double apertureRadius, bool isConvexLens, double focalLength, double voltageInput);
	
	// Set the aperture radius
	void setApertureRadius(double radius);
	
	// Get the aperture radius
	double getApertureRadius() const;
	
	// Set the lens type (convex or concave)
	void setLensType(bool convex);
	
	// Check if the lens is convex
	bool isConvex() const;
	
	// Set the focal length of the lens
	void setFocalLength(double length);
	
	// Get the focal length of the lens
	double getFocalLength() const;
	
	// Set the voltage input
	void setVoltageInput(float voltage);
	
	// Get the voltage input
	float getVoltageInput() const;
	
	// Calculate and print the laser power
	void calculateLaserPower() const;
};


class LaserNode : public ax::Node {
public:
	LaserNode(double apertureRadius, bool isConvexLens, double focalLength, double voltageInput, float laserFrequency);
	virtual ~LaserNode();
	
	static LaserNode* create(double apertureRadius, bool isConvexLens, float focalLength, float voltageInput, float frequency);

	void setApertureRadius(double radius);
	double getApertureRadius() const;
	
	void setLensType(bool convex);
	bool isConvex() const;
	
	void setFocalLength(double length);
	double getFocalLength() const;
	
	void setVoltageInput(float voltage);
	float getVoltageInput() const;
	
	void calculateLaserPower();
	
	void createLaserLight(); // Function to create the PointLight
	void updateLaserLightColor(); // Function to update the laser light's color

	void update(float dt) override;
	
	void simulateOpticalSystem(float dt);
	
	float getAccumulatedVoltage() const;
	void dischargeAccumulatedVoltage(float dischargeAmount);
	
	float getGuiMeasure() const;
private:
	Laser* laser; // Laser instance
	ax::PUParticleSystem3D* laserLight; // PointLight for laser beam

	float accumulatedVoltage;
	float totalTime;  // Added member to keep track of total time
	float timeElapsed;  // Added member to keep track of time elapsed
	float frequency;       // Added member for the laser's frequency

	float maxAccumulatedVoltage = 20;
	
	float voltagePerSecond = 0.0f;
	float guiMeasure = 0.0f;
	
	std::vector<float> currentSamples;

};
