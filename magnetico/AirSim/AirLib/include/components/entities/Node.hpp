#pragma once

#include "common/Common.hpp"

class Node : public std::enable_shared_from_this<Node> {
private:
	msr::airlib::Quaternionr quaternion;
	float currentAngularSpeed; // New variable to store angular speed
	
	msr::airlib::Vector3r position3D;
	
public:
	Node():
	quaternion(msr::airlib::Quaternionr::Identity()),
	currentAngularSpeed(0.0f),
	position3D(msr::airlib::Vector3r(0, 0, 0)){
		
	}
	
	std::shared_ptr<Node> getParent() {
		return this->parent;
	}
	
	msr::airlib::Vector3r getPosition3D(){
		return this->position3D;
	}

	void setPosition3D(const msr::airlib::Vector3r& position){
		this->position3D = position;
	}

	msr::airlib::Vector3r getWorldPosition3D(){
		msr::airlib::Vector3r accumulatedPosition3D = getPosition3D();
		
		auto nodeParent = this->getParent();
		
		while (nodeParent)
		{
			accumulatedPosition3D += nodeParent->getPosition3D();
			nodeParent = nodeParent->getParent();
		}

		return accumulatedPosition3D;
	}
	
	void setRotationQuat(const msr::airlib::Quaternionr& rotation){
		this->quaternion = rotation;
	}
	
	msr::airlib::Quaternionr getRotationQuat(){
		return this->quaternion;
	}
	
	// Getter for angular speed
	float getAngularSpeed() const {
		return this->currentAngularSpeed;
	}
	
	// Setter for angular speed
	void setAngularSpeed(float speed) {
		this->currentAngularSpeed = speed;
	}
	
	void addChild(std::shared_ptr<Node> child){
		child->parent = shared_from_this();
		children.push_back(child);
	}
		
	virtual void update(float){
		updateChildPositions();
	}
	
	std::vector<std::shared_ptr<Node>> getChildren(){
		return this->children;
	}
	
private:
	// Function to update the children's positions based on their parent's position and rotation
	void updateChildPositions() {
		for (auto child : children) {
			// Calculate the child's position relative to the parent's position and rotation
			msr::airlib::Vector3r childRelativePosition = child->getPosition3D();
			msr::airlib::Vector3r childPosition = (quaternion * childRelativePosition);
			
			// Update the child's position
			child->setPosition3D(childPosition);
			
			// Recursively update the child's children positions
			child->updateChildPositions();
		}
	}
	

	std::shared_ptr<Node> parent;
	
	std::vector<std::shared_ptr<Node>> children;
};
