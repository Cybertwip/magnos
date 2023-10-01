
#include "Laser.h"
#include "Utils3d.h"

#include "extensions/Particle3D/PU/PUParticleSystem3D.h"
#include "extensions/Particle3D/PU/PUEmitter.h"

#include <iostream>


// Photonic Oscillator
float photonicOscillator(float frequency, float time, float damping = 0.01f) {
	float amplitude = 1.0f;
	return amplitude * std::exp(-damping * time) * std::sin(2 * M_PI * frequency * time);
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
float currentToVoltage(float current, float resistance = 1.0f) {
	return current * resistance;
}

// Constructor to initialize the Laser object
Laser::Laser(double apertureRadius, bool isConvexLens, double focalLength, double voltageInput) {
	this->apertureRadius = apertureRadius;
	this->isConvexLens = isConvexLens;
	this->focalLength = focalLength;
	this->voltageInput = voltageInput;
}

// Set the aperture radius
void Laser::setApertureRadius(double radius) {
	apertureRadius = radius;
}

// Get the aperture radius
double Laser::getApertureRadius() const {
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
void Laser::setFocalLength(double length) {
	focalLength = length;
}

// Get the focal length of the lens
double Laser::getFocalLength() const {
	return focalLength;
}

// Set the voltage input
void Laser::setVoltageInput(double voltage) {
	voltageInput = voltage;
}

// Get the voltage input
double Laser::getVoltageInput() const {
	return voltageInput;
}

// Calculate and print the laser power
void Laser::calculateLaserPower() const {
	// Simplified power calculation (not accounting for real-world optics)
}

LaserNode::LaserNode(double apertureRadius, bool isConvexLens, double focalLength, double voltageInput, float laserFrequency)
: accumulatedCurrent(0.0f), accumulatedVoltage(0.0f), totalTime(10.0f), timeElapsed(0.0f), frequency(laserFrequency) {
	laser = new Laser(apertureRadius, isConvexLens, focalLength, voltageInput);
	addChild(laser);
	
	// Initialize the laser light (PointLight)
	createLaserLight();
	updateLaserLightColor();
	
	scheduleUpdate();
}

LaserNode::~LaserNode()
{
	AX_SAFE_RELEASE_NULL(_programState);

	delete laser;
}

LaserNode* LaserNode::create(double apertureRadius, bool isConvexLens, float focalLength, float voltageInput, float frequency)
{
	auto node = new LaserNode(apertureRadius, isConvexLens, focalLength, voltageInput, frequency);
	if (node && node->init())
	{
		node->autorelease();
		return node;
	}
	AX_SAFE_DELETE(node);
	return nullptr;
}

void LaserNode::setApertureRadius(double radius)
{
	laser->setApertureRadius(radius);
}

double LaserNode::getApertureRadius() const
{
	return laser->getApertureRadius();
}

void LaserNode::setLensType(bool convex)
{
	laser->setLensType(convex);
}

bool LaserNode::isConvex() const
{
	return laser->isConvex();
}

void LaserNode::setFocalLength(double length)
{
	laser->setFocalLength(length);
}

double LaserNode::getFocalLength() const
{
	return laser->getFocalLength();
}

void LaserNode::setVoltageInput(double voltage)
{
	laser->setVoltageInput(voltage);

	if(getVoltageInput() >= 2.5f){
		laserLight->stopParticleSystem();
		laserLight->startParticleSystem();
	} else {
		laserLight->stopParticleSystem();
	}
	// Update the laser light's color based on the voltage
	//updateLaserLightColor();
}

double LaserNode::getVoltageInput() const
{
	return laser->getVoltageInput();
}

void LaserNode::calculateLaserPower()
{
	laser->calculateLaserPower();
}

void LaserNode::createLaserLight()
{
	auto ps =
	ax::PUParticleSystem3D::create("scripts/electricBeamSystem.pu");
	ps->startParticleSystem();
	
	ps->setRotation3D(ax::Vec3(0, 0, -90));
	this->addChild(ps);

	laserLight = ps;
	
}

void LaserNode::updateLaserLightColor()
{
	auto laserColor = ax::Color3B(255, 0, 0); // Default color (red)
	double voltage = laser->getVoltageInput();
	if (voltage > 2.0) {
		// Increase the green component to make it yellow for higher voltage
		laserColor.g = static_cast<GLubyte>((voltage - 2.0) * 255.0);
	}
	
	laserLight->setColor(laserColor); // Set the laser light's color
}


void LaserNode::update(float dt) {
	// Update the laser light's color based on the amplified voltage
	//updateLaserLightColor();
	
	// Simulate the optical system and accumulate values based on delta time (dt)
	
	if(getVoltageInput() >= 2.5f){
		simulateOpticalSystem(dt);
	}
	
	// Accumulate time elapsed
	timeElapsed += dt;
	
	// Check if it's time to reset the simulation
	if (timeElapsed >= totalTime) {
		timeElapsed = 0.0f;
	}
}
void LaserNode::simulateOpticalSystem(float dt) {
	float damping = 0.01f;
	
	// Calculate light intensity directly
	float lightIntensity = photonicOscillator(frequency, timeElapsed, damping);
	
	// Apply photodetector
	float currentSample = photodetector(lightIntensity);
	
	// Apply optical repeater boost
	float boostedSample = opticalRepeaterBoost(currentSample);
	
	// Convert to voltage
	float boostedVoltageSample = currentToVoltage(boostedSample);
	
	// Accumulate values based on delta time (dt)
	// accumulatedCurrent += boostedSample * dt;
	accumulatedVoltage += boostedVoltageSample * dt;
}

float LaserNode::getAccumulatedVoltage() const {
	return accumulatedVoltage;
}
