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

Car::Car() : acceleration(0.0f), maxSpeed(10.0f), friction(0.02f) {
	// Create car's body, gear box, and gimbals
	std::vector<CustomNode*> wheelsContainer;
	
	carBody = createCarWithWheels(1.25f, 0.1f, 0.2f, wheelsContainer);
	
	frontLeftWheel = wheelsContainer[0];
	frontRightWheel = wheelsContainer[1];
	rearLeftWheel = wheelsContainer[2];
	rearRightWheel = wheelsContainer[3];

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


void Car::accelerate(float value) {
	acceleration = value;
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
	
	// Update the car's position
	this->setPosition3D(newPosition);

}
