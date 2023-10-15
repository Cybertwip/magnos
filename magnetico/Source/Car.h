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
	
//	ax::Node* innerNode;
//	ax::Node* middleNode;
//	ax::Node* outerNode;
	
public:
	Car();
	virtual ~Car();
	
	bool anyLaserStatusOn();
	ax::Node* createLaserSystem(ax::Vec3 position);

	std::vector<LaserNode*> getLasers() const;
	
	float getAcceleration() const;
	float getSpeed() const;
	void charge(float amount);
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
	
};

