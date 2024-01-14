#include "Plane.h"
#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Utils3d.h"

Aircraft::Aircraft() : acceleration(0.0f), maxSpeed(128.0f), friction(0.001f), maxSteeringAngle(15) {
	// Create car's body, gear box, and gimbals
	std::vector<ax::Node*> wheelsContainer;
	
	float planeLength = 1.25f; // Replace with your desired dimensions
	float wingWidth = 1.0f;
	float tailHeight = 0.5f;
	float wheelRadius = 0.2f;
	float wheelWidth = 0.1f;
	
	carBody = createAircraft(planeLength, wingWidth, tailHeight, wheelRadius, wheelWidth, wheelsContainer);
	
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
	
	ballisticVelocity = ax::Vec3(0 * cosf(0), 0 * sinf(0), 0.0f);

}

Aircraft::~Aircraft() {
	// Release any allocated resources
	// ...
}

void Aircraft::update(float dt){
	engine_->setPosition3D(msr::airlib::Vector3r(this->getWorldPosition3D().x, this->getWorldPosition3D().y, this->getWorldPosition3D().z));
	engine_->update(dt);
}

float Aircraft::getSpeed() const{
	return speed;
}

float Aircraft::getAcceleration() const{
	return acceleration;
}

void Aircraft::lift(bool isLifting){
	shouldLift = isLifting;
}

void Aircraft::steer(float angle) {
	// Clamp the steering angle within the valid range
	steeringAngle = std::min(std::max(angle, -maxSteeringAngle), maxSteeringAngle);
}

bool Aircraft::isCalibrating(){
	bool calibrating = false;
	
	for(auto magnos : engine_->getGimbals()){
		
		calibrating = magnos->getCoilSystem().calibrating();

		if(calibrating){
			break;
		}
	}
	
	return calibrating;
}

bool Aircraft::isCollecting(){
	bool collecting = false;
	
	for(auto magnos : engine_->getGimbals()){
		
		collecting = magnos->getCoilSystem().collecting();
		
		if(collecting){
			break;
		}
	}
	
	return collecting;
}

bool Aircraft::anyLaserStatusOn(){
	bool lasersOn = false;
	
	for(auto laser : engine_->getLasers()){
		lasersOn = laser->getVoltageInput() == 2.5f;
		
		if(lasersOn){
			break;
		}
	}
	
	return lasersOn;
}

void Aircraft::brake(float brakePedalInput) {
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

void Aircraft::liftPedal(){
	if(acceleration > 0){
		acceleration = 0;
	}
	
	engine_->decelerate();
}

void Aircraft::accelerate(float throttle) {
	auto powerDrawn = engine_->accelerate(throttle);
	
	// Define Tesla car properties (example values)
	const float maxVoltage = 40.0f; // Maximum voltage output in volts

	const float maxAcceleration = 32.0f; // Maximum acceleration in m/s^2
	
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

void Aircraft::applyFriction() {
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
// Add this function to your Plane class
ax::Vec3 Aircraft::getForwardVector() const {
	// Assuming the forward direction is along the negative z-axis
	return -ax::Vec3(std::sin(carOrientation), 0.0f, -std::cos(carOrientation));
}

void Aircraft::updateMotion(float deltaTime) {
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
	
	
	float steeringDamping = 0.1;
	
	// Calculate the change in orientation (yaw) based on the steering angle
	float yawChange = newSpeed * std::tan(steeringAngle) * deltaTime;
	
	// Update the car's orientation (yaw)
	carOrientation += yawChange * steeringDamping;
	
	// Ensure the carOrientation stays within a valid range (e.g., between 0 and 2 * PI)
	carOrientation = std::fmod(carOrientation, 2 * M_PI);
	if (carOrientation < 0.0f) {
		carOrientation += 2 * M_PI;
	}
	
	//	// Create a quaternion for the existing orientation (carOrientation)
	ax::Quaternion rotationQuatYaw;
	rotationQuatYaw.set(ax::Vec3(0, 1, 0), -carOrientation); // Rotate around the y-axis for yaw


	// Define gravitational force (adjust as needed)
	float gravityForce = 9.8f;
	
	
	// Assuming center of mass offset from the _ballistic's position
	ax::Vec3 centerOfMassOffset = ax::Vec3(0.0f, 0.0f, 0.0f); // Adjust as needed
	
	// Calculate the pitch change based on the vertical velocity and gravity
	float verticalVelocity = ballisticVelocity.y;
	float gravityEffectPitch = gravityForce * centerOfMassOffset.x;
	float pitchChange = atan2(verticalVelocity, newSpeed) - gravityEffectPitch * deltaTime;


	ax::Quaternion rotationQuatPitch;
	rotationQuatPitch.set(ax::Vec3(0, 0, 1), pitchChange); // Rotate around the x-axis for pitch

	
	// Update the position based on the ballistic velocity
	newPosition.x += ballisticVelocity.x * deltaTime;
	newPosition.y += ballisticVelocity.y * deltaTime;
	newPosition.z += ballisticVelocity.z * deltaTime;
	
	ballisticVelocity.y -= gravityForce * deltaTime;

	
	// Check if the object has landed (reached or below ground level)
	if (newPosition.y <= 0) {
		newPosition.y = 0;
		ballisticVelocity = ax::Vec3(); // Stop the ballistic motion when landed
	}

	// Apply lifting force when shouldLift is true
	if (shouldLift) {
		float liftForce = 0.1f; // Adjust for realism
		float lift = liftForce * newSpeed * newSpeed * deltaTime;
		
		float dampingFactor = 0.1f;
		
		// Apply gravity to the lifting force
		float gravityEffect = gravityForce * deltaTime;
		lift -= gravityEffect;
		
		lift *= dampingFactor;
		
		ballisticVelocity.y += lift;
		
	}
	

	ax::Quaternion newRotationQuat = rotationQuatYaw * rotationQuatPitch;

	// Set the new rotation quaternion
	this->setRotationQuat(newRotationQuat);

	
	// Update the car's position
	this->setPosition3D(newPosition);
	
	// Calculate angular velocity for each wheel
	float wheelRadius = 0.2f; // Adjust the wheel radius as needed
	float baseAngularVelocity = newSpeed / wheelRadius * deltaTime;
	
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


std::shared_ptr<EVEngine> Aircraft::getEngine() {
	return engine_;
}
