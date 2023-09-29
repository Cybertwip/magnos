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
	

public:
	Car();
	virtual ~Car();
	
	std::vector<ax::Node*> getGimbals() const;
	
	void accelerate(float value);
	void applyFriction();
	void updateMotion(float deltaTime);

	CREATE_FUNC(Car);
};

