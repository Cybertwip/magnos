#include "BallisticGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Car.h"
#include "Utils3d.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

namespace{
float mpsToKmph(float speedMps) {
	return speedMps * 3.6;
}
}

USING_NS_AX;


BallisticGameState::BallisticGameState() {
	// Initialize private members and resources specific to the car simulation
}

bool BallisticGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init())
	{
		return false;
	}
	
	auto mesh = createCuboid(0.5, 0.5, 2);

	_ballistic = ax::MeshRenderer::create();
	
	_ballistic->setRotation3D(Vec3(0, 90.0f, 0));
	
	_ballistic->addMesh(mesh);
	
	mesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	_ballistic->setMaterial(mesh->getMaterial());
	_ballistic->setTexture("anger.jpg");
	_ballistic->setColor(ax::Color3B::WHITE);
		
	this->addChild(_ballistic);
	
	auto director = Director::getInstance();
	
	director->setClearColor(Color4F(0.0f, 0.0f, 1.0f, 1.0f));
	
	auto plane = createPlane(1024, 1024, 1, 1);
	
	ax::MeshRenderer* planeRenderer = ax::MeshRenderer::create();
	planeRenderer->addMesh(plane);
	auto material = ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false);
	
	Texture2D::TexParams tRepeatParams;  // set texture parameters
	tRepeatParams.magFilter = tRepeatParams.minFilter = backend::SamplerFilter::NEAREST;
	tRepeatParams.sAddressMode                        = backend::SamplerAddressMode::REPEAT;
	tRepeatParams.tAddressMode                        = backend::SamplerAddressMode::REPEAT;
	
	auto checkerTexture = Director::getInstance()->getTextureCache()->addImage("checker.png");
	
	checkerTexture->setTexParameters(tRepeatParams);
	
	planeRenderer->setMaterial(material);
	planeRenderer->setTexture(checkerTexture);
	planeRenderer->setPositionY(-1.25f / 2 - 0.3f);
	this->addChild(planeRenderer);
	
	
	// Initialize ballistic velocity
	ballisticVelocity = Vec3(initialSpeed * cosf(launchAngle), initialSpeed * sinf(launchAngle), 0.0f);

	// Initialize explosion particle system
	explosionParticle = PUParticleSystem3D::create("scripts/explosionSystem.pu"); // Use the filename of your particle system
//	explosionParticle->stopParticleSystem();  // Initially stop the particle system
	explosionParticle->startParticleSystem();
	explosionParticle->pauseParticleSystem();
	this->addChild(explosionParticle);

	return true;
	
}

void BallisticGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	_defaultCamera->setPosition3D(Vec3(1.5f, 1.5f, -1.5f));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0));
}

void BallisticGameState::onMouseMove(Event* event)
{
	EventMouse* e = static_cast<EventMouse*>(event);
	// Get the cursor delta since the last frame
	
	prevCursorX = cursorX;
	prevCursorY = cursorY;
	
	cursorX = e->getDelta().x;
	cursorY = e->getDelta().y;
	
	cursorDeltaX = cursorX - prevCursorX;
	cursorDeltaY = cursorY - prevCursorY;
}


void BallisticGameState::onKeyPressed(EventKeyboard::KeyCode code, Event*)
{
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = true;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		steer = true;
		steerAngle = -6;
	}
	
	
	if(code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = true;
		steerAngle = 6;
	}
	
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = true;
	}
}

void BallisticGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = false;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW || code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = false;
		steerAngle = 0;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = false;
	}
	
}

void BallisticGameState::update(float) {
	float totalDelta = Settings::fixed_delta;
	float deltaTime = totalDelta;
			
	if(accelerate){
		ballisticPosition = Vec3(0.0f, 0.0f, 0.0f);
		ballisticVelocity = Vec3(initialSpeed * cosf(launchAngle), initialSpeed * sinf(launchAngle), 0.0f);
		
		_ballistic->setVisible(true);
		explosionParticle->startParticleSystem();
		explosionParticle->pauseParticleSystem();

	} else {
	}
		
	if(steer){

	} else {
	}
	
	if(brake){
	} else {
	}
	
	Vec3 initialPosition = ballisticPosition;
	Vec3 initialVelocity = ballisticVelocity;
	
	// Calculate the new position based on ballistic trajectory
	float newX = initialPosition.x + initialVelocity.x * deltaTime;
	float newY = initialPosition.y + initialVelocity.y * deltaTime + 0.5f * gravity * deltaTime * deltaTime;
	float newZ = initialPosition.z + initialVelocity.z * deltaTime;
	
	ballisticPosition = Vec3(newX, newY, newZ);
	
	// Calculate the new velocity based on gravity
	ballisticVelocity.x = initialVelocity.x;
	ballisticVelocity.y = initialVelocity.y + gravity * deltaTime;
	ballisticVelocity.z = initialVelocity.z;
	
	// Assuming center of mass offset from the _ballistic's position
	Vec3 centerOfMassOffset = Vec3(-1.0f, 0.0f, 0.0f);  // Adjust as needed
	
	// Calculate the rotation angles based on the trajectory
	float horizontalRotation = atan2(ballisticVelocity.y, ballisticVelocity.x);
	float verticalRotation = atan2(ballisticVelocity.z, sqrt((ballisticVelocity.x * ballisticVelocity.x) + (ballisticVelocity.y * ballisticVelocity.y)));
	float degreesHorizontal = AX_RADIANS_TO_DEGREES(horizontalRotation);
	float degreesVertical = AX_RADIANS_TO_DEGREES(verticalRotation);
	
	// Set the new rotation angles of the ballistic object
	_ballistic->setRotation3D(Vec3(degreesVertical, 90.0f, -degreesHorizontal));
	
	explosionParticle->setPosition3D(ballisticPosition);
	
	explosionParticle->setScale(20);

	// Check for collision with the plane
	if (ballisticPosition.y <= 0.0f) {
		// Trigger explosion effect or handle collision logic here
		
		
		// Stop the ballistic object
		ballisticVelocity = Vec3::ZERO;
		
//		explosionParticle->stopParticleSystem();
		explosionParticle->resumeParticleSystem();


		_ballistic->setVisible(false);
//		// For simplicity, let's just reset the position and velocity
	} else {
		// Set the new position of the ballistic object
		_ballistic->setPosition3D(ballisticPosition);
	}


	// Get the ballistic's position
	Vec3 ballisticPosition = _ballistic->getPosition3D();
	
	// Calculate new camera rotation angles based on normalized cursor deltas
	horizontalAngle += cursorDeltaX * sensitivity;
	verticalAngle -= cursorDeltaY * sensitivity;
	
	// Define the vertical angle constraints (adjust as needed)
	float minVerticalAngle = AX_DEGREES_TO_RADIANS(0); // Minimum vertical angle (degrees)
	float maxVerticalAngle = AX_DEGREES_TO_RADIANS(60.0f); // Maximum vertical angle (degrees)
	
	// Clamp the vertical angle within the specified range
	verticalAngle = std::min(std::max(verticalAngle, minVerticalAngle), maxVerticalAngle);
	
	// Calculate the new camera position relative to the car
	float distanceFromCar = 3.0f; // Adjust the distance as needed
	float cameraHeight = 2.0f;   // Adjust the height as needed
	
	// Calculate the camera's offset from the car based on angles
	float horizontalOffset = distanceFromCar * sinf(horizontalAngle);
	float verticalOffset = distanceFromCar * cosf(horizontalAngle) * sinf(verticalAngle);
	float depthOffset = distanceFromCar * cosf(horizontalAngle) * cosf(verticalAngle);
	
	Vec3 cameraOffset(horizontalOffset, cameraHeight + verticalOffset, depthOffset);
	
	// Calculate the new camera position
	Vec3 newPosition = ballisticPosition + cameraOffset;
	
	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(ballisticPosition);
	
	cursorDeltaX = 0;
	cursorDeltaY = 0;
	
}

void BallisticGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Ballistic");
	
	ImGui::End();
	
	
}
