
#include "Physics.h"
#include "Settings.h"

#include "components/systems/CoilSystem.hpp"

RigidBody::RigidBody(float mass, ax::Vec3 position, ax::Vec3 velocity, ax::Vec3 acceleration, float restitution, float friction) :
_mass(mass),
_position(position),
_lastPosition(position),
_velocity(velocity),
_acceleration(acceleration),
_restitution(restitution),
_friction(friction)
{}

RigidBody::~RigidBody() {}

void RigidBody::update(float deltaTime) {
	// Calculate the displacement based on the current velocity and time step
	ax::Vec3 displacement = _velocity * deltaTime;
	
	// Update the position based on the displacement
	_position += displacement;
		
	// Update the last measured position
	_lastPosition = _position;
	
	// Update the velocity based on the current acceleration
	_velocity += _acceleration * deltaTime;

}

void RigidBody::applyForce(ax::Vec3 force) {
	// Apply the force to the acceleration
	_acceleration += force / _mass;
}

ax::Vec3 RigidBody::getPosition() const {
	return _position;
}

ax::Vec3 RigidBody::getVelocity() const {
	return (_position - _lastPosition) / Settings::global_delta;
}

ax::Vec3 RigidBody::getAcceleration() const {
	return _acceleration;
}

float RigidBody::getMass() const {
	return _mass;
}

float RigidBody::getRestitution() const {
	return _restitution;
}

float RigidBody::getFriction() const {
	return _friction;
}

ax::Vec3 RigidBody::getCenterOfMassPosition() const {
	// Return the position of the rigid body's center of mass
	return _position;
}

ax::Vec3 RigidBody::getLinearVelocity() const {
	// Return the linear velocity of the rigid body
	return _velocity;
}

float RigidBody::getInvMass() const {
	// Return the inverse mass of the rigid body
	if (_mass == 0) {
		return 0;
	}
	return 1.0f / _mass;
}

RigidBodyConstructionInfo::RigidBodyConstructionInfo(float mass, ax::Vec3 position, ax::Vec3 velocity, ax::Vec3 acceleration, float restitution, float friction) :
_mass(mass),
_position(position),
_velocity(velocity),
_acceleration(acceleration),
_restitution(restitution),
_friction(friction)
{}

RigidBodyConstructionInfo::~RigidBodyConstructionInfo() {}

RigidBody* RigidBodyConstructionInfo::createRigidBody() const {
	// Create a new rigid body with the construction info
	return new RigidBody(_mass, _position, _velocity, _acceleration, _restitution, _friction);
}

CollisionShape::~CollisionShape() {}

bool CollisionShape::checkCollision(const RigidBody& body) const {
	// Call the derived class's implementation of _checkCollision
	return _checkCollision(body);
}

CylinderShape::CylinderShape(float radius, float height) :
_radius(radius),
_height(height)
{}

CylinderShape::~CylinderShape() {}

bool CylinderShape::_checkCollision(const RigidBody& body) const {
	// Check if the body's position is within the cylinder's bounds
	ax::Vec3 position = body.getPosition();
	if (position.y < 0 || position.y > _height) {
		return false;
	}
	float distance_from_center = sqrt(pow(position.x, 2) + pow(position.z, 2));
	if (distance_from_center > _radius) {
		return false;
	}
	return true;
}

void CylinderShape::calculateLocalInertia(float mass, ax::Vec3& inertia) const {
	// Calculate the local inertia of the cylinder
	float radius_squared = _radius * _radius;
	float height_squared = _height * _height;
	float ixx = (mass / 12.0f) * (3.0f * radius_squared + height_squared);
	float iyy = (mass / 2.0f) * radius_squared;
	float izz = ixx;
	inertia = ax::Vec3(ixx, iyy, izz);
}
