#pragma once
#include "CustomNode.h"

#include <axmol.h>

#include <vector>

class Car : public ax::Node {
private:
	ax::Node* carBody;
	ax::Node* gearBox;
	std::vector<ax::Node*> gimbals;
	CustomNode* frontLeftWheel;
	CustomNode* frontRightWheel;
	CustomNode* rearLeftWheel;
	CustomNode* rearRightWheel;
	

	// Member variables for acceleration and friction
	float acceleration;
	float maxSpeed;
	float friction;
	float steeringAngle;
	float maxSteeringAngle; // Maximum steering angle in radians
	float rotationAngle = 0;

public:
	Car();
	virtual ~Car();
	
	std::vector<ax::Node*> getGimbals() const;
	
	void steer(float angle); // Function to set the steering angle
	void accelerate(float value);
	void applyFriction();
	void updateMotion(float deltaTime);

	CREATE_FUNC(Car);
	
};

