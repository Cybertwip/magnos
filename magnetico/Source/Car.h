#pragma once
#include "CustomNode.h"
#include "Battery.h"

#include <axmol.h>

#include <vector>

class LaserNode;


class Car : public ax::Node {
private:
	ax::Node* carBody;
	ax::Node* engineBox;
	std::vector<ax::Node*> gimbals;
	ax::Node* frontLeftWheel;
	ax::Node* frontRightWheel;
	ax::Node* rearLeftWheel;
	ax::Node* rearRightWheel;
	ax::Node* rearSuspension;
	std::vector<LaserNode*> lasers;

	// Member variables for acceleration and friction
	float acceleration;
	float maxSpeed;
	float friction;
	float steeringAngle;
	float maxSteeringAngle; // Maximum steering angle in radians
	float rotationAngle = 0;
	float speed = 0;
	float mass = 150;
	float carOrientation = 0;
	float brakePower = 0;
	
	
	RechargeableBattery battery = RechargeableBattery(9.0f, 10.0f, 10.0f, 0.9f, 0.95f);
public:
	Car();
	virtual ~Car();
	
	ax::Node* createLaserSystem(ax::Vec3 position);

	std::vector<ax::Node*> getGimbals() const;
	std::vector<LaserNode*> getLasers() const;
	
	float getAcceleration() const;
	float getSpeed() const;
	void charge(float amount);
	void brake(float amount);
	void steer(float angle); // Function to set the steering angle
	void accelerate(float value);
	void applyFriction();
	void updateMotion(float deltaTime);
	
	RechargeableBattery& getBattery();

	CREATE_FUNC(Car);
	
};

