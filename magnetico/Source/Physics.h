
#pragma once

#include <axmol.h>


class RigidBody {
public:
	RigidBody(float mass, ax::Vec3 position, ax::Vec3 velocity, ax::Vec3 acceleration, float restitution, float friction);
	virtual ~RigidBody();
	
	void update(float deltaTime);
	void applyForce(ax::Vec3 force);
	
	ax::Vec3 getPosition() const;
	ax::Vec3 getVelocity() const;
	ax::Vec3 getAcceleration() const;
	float getMass() const;
	float getRestitution() const;
	float getFriction() const;
	ax::Vec3 getCenterOfMassPosition() const;
	ax::Vec3 getLinearVelocity() const;
	float getInvMass() const;

	
private:
	ax::Vec3 _position;
	ax::Vec3 _velocity;
	ax::Vec3 _acceleration;
	float _mass;
	float _restitution;
	float _friction;
};

class RigidBodyConstructionInfo {
public:
	RigidBodyConstructionInfo(float mass, ax::Vec3 position, ax::Vec3 velocity, ax::Vec3 acceleration, float restitution, float friction);
	virtual ~RigidBodyConstructionInfo();
	
	RigidBody* createRigidBody() const;
	
private:
	float _mass;
	ax::Vec3 _position;
	ax::Vec3 _velocity;
	ax::Vec3 _acceleration;
	float _restitution;
	float _friction;
	ax::Vec3 _inertia;
};

class CollisionShape {
public:
	virtual ~CollisionShape();
	
	bool checkCollision(const RigidBody& body) const;
	virtual void calculateLocalInertia(float mass, ax::Vec3& inertia) const = 0;
	
protected:
	virtual bool _checkCollision(const RigidBody& body) const = 0;
};

class CylinderShape : public CollisionShape {
public:
	CylinderShape(float radius, float height);
	virtual ~CylinderShape();
	void calculateLocalInertia(float mass, ax::Vec3& inertia) const override;
	
protected:
	virtual bool _checkCollision(const RigidBody& body) const override;
	
private:
	float _radius;
	float _height;
};
