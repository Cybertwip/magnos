#include "Car.h"
#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Utils3d.h"

Car::Car() : acceleration(0.0f), maxSpeed(116.0f), friction(0.001f), maxSteeringAngle(15) {
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
	
	carBody->addChild(engineBox);
	
	this->addChild(carBody);
	
	engine_ = std::make_shared<EVEngine>();
	
	engine_->setPosition3D(msr::airlib::Vector3r(0, 0, 0));
	engine_->init(ax::FileUtils::getInstance()->getWritablePath());
//
//	auto magnos = engine_->getGimbals()[0];
//
//	auto innerNode = ax::Node::create();
//
//	for(auto child : magnos->innerNode->getChildren()){
//		auto cube = ax::MeshRenderer::create();
//
//		auto cubeMesh = createCube(0.01f);
//
//		cube->addMesh(cubeMesh);
//
//		cubeMesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
//
//		cube->setTexture("gray.jpg");
//
//		innerNode->addChild(cube);
//
//		cube->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
//
//	auto middleNode = ax::Node::create();
//
//
//	for(auto child : magnos->middleNode->getChildren()){
//		auto cube = ax::MeshRenderer::create();
//
//		auto cubeMesh = createCube(0.01f);
//
//		cube->addMesh(cubeMesh);
//
//		cubeMesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
//
//		cube->setTexture("gray.jpg");
//
//		middleNode->addChild(cube);
//
//		cube->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
//
//	auto outerNode = ax::Node::create();
//
//	for(auto child : magnos->outerNode->getChildren()){
//		auto cube = ax::MeshRenderer::create();
//
//		auto cubeMesh = createCube(0.01f);
//
//		cube->addMesh(cubeMesh);
//
//		cubeMesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
//
//		cube->setTexture("gray.jpg");
//
//		outerNode->addChild(cube);
//
//		cube->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
//
//	this->innerNode = innerNode;
//	this->middleNode = middleNode;
//	this->outerNode = outerNode;
//
//	this->addChild(innerNode);
//	this->addChild(middleNode);
//	this->addChild(outerNode);

//	for(auto child : magnos->middleNode->getChildren()){
//		auto cube = ax::MeshRenderer::create();
//
//		auto cubeMesh = createCube(0.01f);
//
//		cube->addMesh(cubeMesh);
//
//		cubeMesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
//
//		cube->setTexture("gray.jpg");
//
//		this->addChild(cube);
//
//		cube->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
	
	
}

Car::~Car() {
	// Release any allocated resources
	// ...
}

void Car::update(float dt){
//
//	auto magnos = engine_->getGimbals()[0];
//
//	for(size_t i = 0; i<magnos->innerNode->getChildren().size(); ++i){
//
//		auto child = magnos->innerNode->getChildren()[i];
//
//		this->innerNode->getChildren().at(i)->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
//
//
//	for(size_t i = 0; i<magnos->middleNode->getChildren().size(); ++i){
//
//		auto child = magnos->middleNode->getChildren()[i];
//
//		this->middleNode->getChildren().at(i)->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}
//
//
//	for(size_t i = 0; i<magnos->outerNode->getChildren().size(); ++i){
//
//		auto child = magnos->outerNode->getChildren()[i];
//
//		this->outerNode->getChildren().at(i)->setPosition3D(ax::Vec3(child->getPosition3D().x(), child->getPosition3D().y(), child->getPosition3D().z()));
//	}

	engine_->setPosition3D(msr::airlib::Vector3r(this->getWorldPosition3D().x, this->getWorldPosition3D().y, this->getWorldPosition3D().z));
	engine_->update(dt);
}

float Car::getSpeed() const{
	return speed;
}

float Car::getAcceleration() const{
	return acceleration;
}

void Car::steer(float angle) {
	// Clamp the steering angle within the valid range
	steeringAngle = std::min(std::max(angle, -maxSteeringAngle), maxSteeringAngle);
}

bool Car::isCalibrating(){
	bool calibrating = false;
	
	for(auto magnos : engine_->getGimbals()){
		
		calibrating = magnos->getCoilSystem().calibrating();

		if(calibrating){
			break;
		}
	}
	
	return calibrating;
}


bool Car::isCollecting(){
	bool collecting = false;
	
	for(auto magnos : engine_->getGimbals()){
		
		collecting = magnos->getCoilSystem().collecting();
		
		if(collecting){
			break;
		}
	}
	
	return collecting;
}

bool Car::anyLaserStatusOn(){
	bool lasersOn = false;
	
	for(auto laser : engine_->getLasers()){
		lasersOn = laser->getVoltageInput() == 2.5f;
		
		if(lasersOn){
			break;
		}
	}
	
	return lasersOn;
}

void Car::brake(float brakePedalInput) {
	// Calculate the braking force based on brake pedal input
	float maxBrakeForce = 50000.0f; // Adjust for realism
	float brakingForce = maxBrakeForce * brakePedalInput;
	
	// Apply the braking force to decelerate the car
	brakePower = brakingForce / mass;
	
	// Ensure brakePower does not go below zero (prevents negative braking)
	if (brakePower < 0.0f) {
		brakePower = 0.0f;
	}
	
	// Check if the brake pedal is pressed (positive input)
	if (brakePedalInput > 0.0f) {
		accelerate(-acceleration);
	}
}

void Car::liftPedal(){
	if(acceleration > 0){
		acceleration = 0;
	}
	
	engine_->decelerate();
}

void Car::accelerate(float throttle) {
	auto powerDrawn = engine_->accelerate(throttle);
	
	// Define Tesla car properties (example values)
//	const float maxVoltage = 400.0f; // Maximum voltage output in volts
	const float maxVoltage = 40.0f; // Maximum voltage output in volts

//	const float maxAcceleration = 2.00f; // Maximum acceleration in m/s^2
	const float maxAcceleration = 8.0f; // Maximum acceleration in m/s^2
	
	// Scale the voltage input to acceleration based on Tesla car properties
	acceleration += (powerDrawn / maxVoltage) * maxAcceleration;
	
	acceleration *= throttle;
	
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

std::shared_ptr<EVEngine> Car::getEngine() {
	return engine_;
}
