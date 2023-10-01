#include "Car.h"
#include "MaritimeGimbal3D.h"
#include "Utils3d.h"
#include "Laser.h"


MaritimeGimbal3D* createGimbal(int id, ax::Node* parent, ax::Vec3 position){
	auto gimbal = MaritimeGimbal3D::create();
	gimbal->loadData(id);
	gimbal->setupGui();
	gimbal->setPosition3D(position);
	parent->addChild(gimbal);
	gimbal->attachPinball();
	return gimbal;
}

Car::Car() : acceleration(0.0f), maxSpeed(10.0f), friction(0.02f), maxSteeringAngle(15) {
	// Create car's body, gear box, and gimbals
	std::vector<ax::Node*> wheelsContainer;
	
	float carDimension = 1.25f;
	
	carBody = createCarWithWheels(carDimension, 0.1f, 0.2f, wheelsContainer);
	
	frontLeftWheel = wheelsContainer[0];
	frontRightWheel = wheelsContainer[1];
	rearLeftWheel = wheelsContainer[2];
	rearRightWheel = wheelsContainer[3];
	
	rearSuspension = wheelsContainer[4];

	auto engineBoxMesh = createCube(0.45f);
	auto engineBoxRenderer = ax::MeshRenderer::create();
	engineBoxRenderer->addMesh(engineBoxMesh);
	engineBoxRenderer->setPosition3D(ax::Vec3(0.65f, 0, 0));
	engineBoxRenderer->setRotation3D(ax::Vec3(0, 180, 0));
	engineBoxRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	engineBoxRenderer->setTexture("kitty.jpg");
	engineBoxRenderer->setOpacity(50);
	engineBoxRenderer->setScaleX(1);
	engineBoxRenderer->setScaleY(1);
	engineBoxRenderer->setScaleZ(1);

	engineBox = engineBoxRenderer;
	
	// Create and position gimbals
	engineBox->setPosition3D(ax::Vec3(0.65f, 0, 0));
	engineBox->setRotation3D(ax::Vec3(0, 180, 0));
	
	gimbals.push_back(createGimbal(1, engineBox, ax::Vec3(-0.25, 0, 0)));
	gimbals.push_back(createGimbal(2, engineBox, ax::Vec3(0.25, 0, 0)));
	gimbals.push_back(createGimbal(3, engineBox, ax::Vec3(0, 0, -0.25)));
	gimbals.push_back(createGimbal(4, engineBox, ax::Vec3(0, 0, 0.25)));

	gimbals.push_back(createGimbal(5, engineBox, ax::Vec3(0, 0.25f, 0)));

	gimbals.push_back(createGimbal(6, engineBox, ax::Vec3(0, -0.25f, 0)));

	// Create a LaserNode instance
	laserNode = LaserNode::create(0.02f, true, 0.1f, 0.0f, 30.0f); // Set your laser parameters
	
	// Set the position and add the LaserNode to the scene
	laserNode->setPosition3D(ax::Vec3(-0.2f, 0.0f, 0.0f)); // Adjust the position as needed
	
	auto laserEmitter = createCube(0.1f);
	auto laserReceiver = createCube(0.1f);
	auto laserSystem = ax::Node::create();
	
	auto emitter = ax::MeshRenderer::create();
	auto receiver = ax::MeshRenderer::create();
	emitter->addMesh(laserEmitter);
	receiver->addMesh(laserReceiver);
	emitter->setPosition3D(ax::Vec3(engineBox->getPositionX() - 0.2f, 0, 0));
	receiver->setPosition3D(ax::Vec3(emitter->getPositionX() - 1.5f, 0, 0));
	emitter->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	receiver->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	emitter->setTexture("black.jpg");
	receiver->setTexture("black.jpg");

	emitter->addChild(laserNode);
	laserSystem->addChild(emitter);
	laserSystem->addChild(receiver);
	
	carBody->addChild(laserSystem);
	carBody->addChild(engineBox);
	this->addChild(carBody);
	
}

Car::~Car() {
	// Release any allocated resources
	// ...
}

float Car::getSpeed() const{
	return speed;
}

float Car::getAcceleration() const{
	return acceleration;
}

std::vector<ax::Node*> Car::getGimbals() const {
	return this->gimbals;
}

LaserNode* Car::getLaserNode() const {
	return laserNode;
}

void Car::steer(float angle) {
	// Clamp the steering angle within the valid range
	steeringAngle = std::min(std::max(angle, -maxSteeringAngle), maxSteeringAngle);
}

void Car::brake(float brakePedalInput) {
	// Calculate the braking force based on brake pedal input
	float maxBrakeForce = 500.0f; // Adjust for realism
	float brakingForce = maxBrakeForce * brakePedalInput;
	
	// Apply the braking force to decelerate the car
	brakePower = brakingForce / mass;
	
	// Ensure brakePower does not go below zero (prevents negative braking)
	if (brakePower < 0.0f) {
		brakePower = 0.0f;
	}
}

void Car::charge(float laserInput){
	laserNode->setVoltageInput(laserInput);
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
	// Calculate the car's new speed based on acceleration and time
	
	// Calculate the car's new speed based on acceleration and time
	float newSpeed = speed + acceleration * deltaTime;
	
	// Limit the speed to the maximum allowed
	if (newSpeed > maxSpeed) {
		newSpeed = maxSpeed;
	} else if (newSpeed < -maxSpeed) {
		newSpeed = -maxSpeed;
	}
	
	// Apply braking force to decelerate the car
	float brakingForce = brakePower; // Use positive acceleration for braking
		
	// Calculate the new speed after braking
	newSpeed -= brakingForce * deltaTime;
	
	// Ensure the speed doesn't go below zero
	if (newSpeed < 0.0f) {
		newSpeed = 0.0f;
	}

	
	// Calculate the car's new position based on the updated speed
	ax::Vec3 newPosition = this->getPosition3D();
	
	// Calculate linear movement (forward movement) along the x-axis
	float linearMovementX = newSpeed * std::cos(carOrientation) * deltaTime;
	
	// Calculate lateral movement (sideways movement) along the x-axis
	float lateralMovementX = newSpeed * std::sin(carOrientation) * deltaTime;

	// Update the car's position
	newPosition.x += linearMovementX;
	newPosition.z += lateralMovementX;

	// Update the car's position
	this->setPosition3D(newPosition);
	
	// Calculate the change in orientation (yaw) based on the steering angle
	float yawChange = newSpeed * std::tan(steeringAngle) * deltaTime;
	
	// Update the car's orientation (yaw)
	carOrientation += yawChange;
	
	// Ensure the carOrientation stays within a valid range (e.g., between 0 and 2 * PI)
	carOrientation = std::fmod(carOrientation, 2 * M_PI);
	if (carOrientation < 0.0f) {
		carOrientation += 2 * M_PI;
	}
	
	this->setRotation3D(ax::Vec3(this->getRotation3D().x, AX_RADIANS_TO_DEGREES(-carOrientation), this->getRotation3D().z));

	
	// Calculate angular velocity for each wheel
	float wheelRadius = 0.2f; // Adjust the wheel radius as needed
	float baseAngularVelocity = newSpeed / wheelRadius * deltaTime;
//	float frontLeftAngularVelocity = baseAngularVelocity * (1.0f + steeringAngle);
//	float frontRightAngularVelocity = baseAngularVelocity * (1.0f - steeringAngle);
	
	// Apply rotations to the wheels
	rotationAngle += AX_RADIANS_TO_DEGREES(baseAngularVelocity);
	
	// Calculate the rotation quaternions for the wheels
	ax::Quaternion frontLeftRotation;
	frontLeftRotation.set(ax::Vec3(0, 0, 1), AX_DEGREES_TO_RADIANS(-rotationAngle));
	
	ax::Quaternion frontRightRotation;
	frontRightRotation.set(ax::Vec3(0, 0, 1), AX_DEGREES_TO_RADIANS(-rotationAngle));
	
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

	rearSuspension->setRotation3D(rearRightWheel->getRotation3D() + ax::Vec3(0, 0, AX_RADIANS_TO_DEGREES(baseAngularVelocity)));

	// Update the car's speed
	speed = newSpeed;
}
