#pragma once
#include "CustomNode.h"
#include "components/Battery.hpp"
#include "Settings.h"

#include <axmol.h>

#include <vector>

class EVEngine;
class LaserNode;

class Car : public ax::Node {
private:
	std::shared_ptr<EVEngine> engine_;
	ax::Node* carBody;
	ax::Node* engineBox;
	ax::Node* frontLeftWheel;
	ax::Node* frontRightWheel;
	
	// Member variables for acceleration and friction
	float acceleration;
	float maxSpeed;
	float friction;
	float steeringAngle;
	float maxSteeringAngle; // Maximum steering angle in radians
	float rotationAngle = 0;
	float speed = 0;
	float mass = 1500;
	float carOrientation = 0;
	float brakePower = 0;
	
//	ax::Node* innerNode;
//	ax::Node* middleNode;
//	ax::Node* outerNode;
	
public:
	Car();
	virtual ~Car();
	
	bool anyLaserStatusOn();
	
	float getAcceleration() const;
	float getSpeed() const;
	void charge(float amountl, float delta);
	void brake(float amount);
	void steer(float angle); // Function to set the steering angle
	void liftPedal();
	void accelerate(float value);
	void applyFriction();
	void updateMotion(float deltaTime);
	
	bool isCalibrating();
	bool isCollecting();
	
	std::shared_ptr<EVEngine> getEngine();
	
	void update(float dt) override;

	CREATE_FUNC(Car);
	
	ax::Node* rearLeftWheel;
	ax::Node* rearRightWheel;
	ax::Node* rearSuspension;
};

