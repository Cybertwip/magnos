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

Car::Car() {
	// Create car's body, gear box, and gimbals
	carBody = createCarWithWheels(1.25f, 0.1f, 0.2f);
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
