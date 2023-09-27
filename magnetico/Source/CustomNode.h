#pragma once

#include <axmol.h>

class CustomNode : public ax::MeshRenderer {
private:
	ax::Quaternion previousRotation;
	float currentAngularSpeed; // New variable to store angular speed
	
public:
	CustomNode();
	
	// Getter for angular speed
	float getAngularSpeed() const;
	
	// Setter for angular speed
	void setAngularSpeed(float speed);
	
	CREATE_FUNC(CustomNode);
};
