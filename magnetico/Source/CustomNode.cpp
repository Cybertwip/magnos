#include "CustomNode.h"

CustomNode::CustomNode() : previousRotation(ax::Quaternion()), currentAngularSpeed(0.0f) {
    
}

// Getter for angular speed
float CustomNode::getAngularSpeed() const {
    return currentAngularSpeed;
}

// Setter for angular speed
void CustomNode::setAngularSpeed(float speed) {
    currentAngularSpeed = speed;
}