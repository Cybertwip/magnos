#include "Car.h"
#include "MaritimeGimbal3D.h"
#include "Utils3d.h"

MaritimeGimbal3D* createGimbal(ax::Node* parent, ax::Vec3 position){
	auto gimbal = MaritimeGimbal3D::create();
	gimbal->setPosition3D(position);
	parent->addChild(gimbal);
	gimbal->attachPinball();
	return gimbal;
}

Car::Car() : acceleration(0.0f), maxSpeed(10.0f), friction(0.02f), maxSteeringAngle(15) {
	// Create car's body, gear box, and gimbals
	std::vector<CustomNode*> wheelsContainer;
	
	carBody = createCarWithWheels(1.25f, 0.1f, 0.2f, wheelsContainer);
	
	frontLeftWheel = wheelsContainer[3];
	frontRightWheel = wheelsContainer[1];
	rearLeftWheel = wheelsContainer[2];
	rearRightWheel = wheelsContainer[0];

	auto gearBoxMesh = createCube(0.45f);
	auto gearBoxRenderer = ax::MeshRenderer::create();
	gearBoxRenderer->addMesh(gearBoxMesh);
	gearBoxRenderer->setPosition3D(ax::Vec3(0.65f, 0, 0));
	gearBoxRenderer->setRotation3D(ax::Vec3(0, 180, 0));
	gearBoxRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	gearBoxRenderer->setTexture("kitty.jpg");
	gearBoxRenderer->setOpacity(50);
	gearBoxRenderer->setScaleX(1);
	gearBoxRenderer->setScaleY(1);
	gearBoxRenderer->setScaleZ(1);

	gearBox = gearBoxRenderer;
	
	// Create and position gimbals
	gearBox->setPosition3D(ax::Vec3(0.65f, 0, 0));
	gearBox->setRotation3D(ax::Vec3(0, 180, 0));
	
	gimbals.push_back(createGimbal(gearBox, ax::Vec3(-0.25, 0, 0)));
	gimbals.push_back(createGimbal(gearBox, ax::Vec3(0.25, 0, 0)));
	gimbals.push_back(createGimbal(gearBox, ax::Vec3(0, 0, -0.25)));
	gimbals.push_back(createGimbal(gearBox, ax::Vec3(0, 0, 0.25)));
	
	// Add car components to the Car node
	this->addChild(gearBox);
	this->addChild(carBody);
}

Car::~Car() {
	// Release any allocated resources
	// ...
}


std::vector<ax::Node*> Car::getGimbals() const {
	return this->gimbals;
}

void Car::steer(float angle) {
	// Clamp the steering angle within the valid range
	steeringAngle = std::min(std::max(angle, -maxSteeringAngle), maxSteeringAngle);
}


void Car::accelerate(float voltage) {
	// Define Tesla car properties (example values)
//	const float maxVoltage = 400.0f; // Maximum voltage output in volts
	const float maxVoltage = 40.0f; // Maximum voltage output in volts

//	const float maxAcceleration = 2.00f; // Maximum acceleration in m/s^2
	const float maxAcceleration = 8.0f; // Maximum acceleration in m/s^2
	
	// Scale the voltage input to acceleration based on Tesla car properties
	acceleration = (voltage / maxVoltage) * maxAcceleration;
	
	// Ensure acceleration is within the valid range
	if (acceleration > maxAcceleration) {
		acceleration = maxAcceleration;
	} else if (acceleration < -maxAcceleration) {
		acceleration = -maxAcceleration;
	}
}

void Car::applyFriction() {
	// Apply friction to slow down the car's speed
	if (acceleration > 0.0f) {
		// Apply friction when accelerating forward
		acceleration -= friction;
		if (acceleration < 0.0f) {
			acceleration = 0.0f;  // Ensure acceleration doesn't go negative
		}
	} else if (acceleration < 0.0f) {
		// Apply friction when accelerating backward
		acceleration += friction;
		if (acceleration > 0.0f) {
			acceleration = 0.0f;  // Ensure acceleration doesn't go positive
		}
	}
	
	// Ensure the car's speed does not exceed the maximum speed
	if (acceleration > maxSpeed) {
		acceleration = maxSpeed;
	} else if (acceleration < -maxSpeed) {
		acceleration = -maxSpeed;
	}
}

void Car::updateMotion(float deltaTime) {
	applyFriction();
	
	// Calculate the car's new position based on acceleration and time
	// You can use a physics engine or simple equations to update the position and speed.
	// For simplicity, let's assume constant velocity for this example.
	float velocity = acceleration * deltaTime;
	ax::Vec3 newPosition = this->getPosition3D() + ax::Vec3(velocity, 0, 0);
	
	// Calculate lateral (sideways) movement based on steering
	float lateralMovement = velocity * -steeringAngle * deltaTime;
	
	// Update the car's lateral position
	newPosition += ax::Vec3(0, 0, lateralMovement);
	
	// Update the car's position
	this->setPosition3D(newPosition);

	// Calculate angular velocity for each wheel
	float wheelRadius = 0.2f; // Adjust the wheel radius as needed
	float baseAngularVelocity = velocity / wheelRadius;
	float frontLeftAngularVelocity = baseAngularVelocity * (1.0f + steeringAngle);
	float frontRightAngularVelocity = baseAngularVelocity * (1.0f - steeringAngle);
	
	// Apply rotations to the wheels
	rotationAngle += AX_RADIANS_TO_DEGREES(baseAngularVelocity);
	
	// Calculate the rotation quaternions for the wheels
	ax::Quaternion frontLeftRotation;
	frontLeftRotation.set(ax::Vec3(0, 0, 1), AX_DEGREES_TO_RADIANS(rotationAngle));
	
	ax::Quaternion frontRightRotation;
	frontRightRotation.set(ax::Vec3(0, 0, 1), AX_DEGREES_TO_RADIANS(rotationAngle));
	
	// Combine the rotations with the steering angle
	ax::Quaternion frontLeftRotationSteer;
	frontLeftRotationSteer.set(ax::Vec3(0, 1, 0), -steeringAngle);
	
	ax::Quaternion frontRightRotationSteer;
	frontRightRotationSteer.set(ax::Vec3(0, 1, 0), -steeringAngle);
	
	// Set the rotation of the front left wheel using the combined quaternion
	frontLeftWheel->setRotationQuat(frontLeftRotationSteer * frontLeftRotation);
	
	// Set the rotation of the front right wheel using the combined quaternion
	frontRightWheel->setRotationQuat(frontRightRotationSteer * frontRightRotation);

	rearLeftWheel->setRotation3D(rearLeftWheel->getRotation3D() + ax::Vec3(0, 0, AX_RADIANS_TO_DEGREES(baseAngularVelocity)));
	rearRightWheel->setRotation3D(rearRightWheel->getRotation3D() + ax::Vec3(0, 0, AX_RADIANS_TO_DEGREES(baseAngularVelocity)));


}
