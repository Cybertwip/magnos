#include "BallisticGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Plane.h"
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
	
//	_ballistic = ax::MeshRenderer::create();
	
//	_ballistic->setRotation3D(Vec3(0, 90.0f, 0));
	
//	_ballistic->addMesh(mesh);
	
//	mesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
//	_ballistic->setMaterial(mesh->getMaterial());
//	_ballistic->setTexture("anger.jpg");
//	_ballistic->setColor(ax::Color3B::WHITE);
//		
//	this->addChild(_ballistic);
	
	_ballistic = Aircraft::create();
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
	if(_ballistic->isCalibrating()){
		return;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = true;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_UP_ARROW){
		lift = true;
		liftAmount = 1.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		lift = true;
		liftAmount = -1.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		roll = true;
		rollAmount = -1.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		roll = true;
		rollAmount = 1.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_D){
		steer = true;
		steerAngle = -6;
	}
	
	
	if(code ==  EventKeyboard::KeyCode::KEY_A){
		steer = true;
		steerAngle = 6;
	}
	
	
	if(code == EventKeyboard::KeyCode::KEY_S){
		brake = true;
	}
}

void BallisticGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{
	
	if(_ballistic->isCalibrating()){
		return;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = false;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_UP_ARROW || code ==  EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		lift = false;
		liftAmount = 0.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_LEFT_ARROW || code ==  EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		roll = false;
		rollAmount = 0.0f;
	}

	if(code == EventKeyboard::KeyCode::KEY_D || code ==  EventKeyboard::KeyCode::KEY_A){
		steer = false;
		steerAngle = 0;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_S){
		brake = false;
	}
	
}

void BallisticGameState::update(float) {
	float totalDelta = Settings::fixed_delta;
	
	bool anyDataCollectionMode = _ballistic->isCollecting();
	
	if(accelerate || anyDataCollectionMode){
		
		_ballistic->getEngine()->setEngineConsumption(400);
		
		_ballistic->accelerate(0.1f); // @TODO throttle
		
	} else {
		_ballistic->getEngine()->setEngineConsumption(0);
		
		_ballistic->liftPedal();
	}
	
	_ballistic->update(totalDelta);
	
	if(lift){
		_ballistic->pitch(liftAmount);
	} else {
		_ballistic->pitch(0.0f);
	}

	if(roll){
		_ballistic->roll(rollAmount);
	} else {
		_ballistic->roll(0.0f);
	}

	if(steer){
		_ballistic->steer(steerAngle);
	} else {
		_ballistic->steer(0);
	}
	
	if(brake){
		float brakePedalInput = 1.0f; // Adjust as needed
		_ballistic->brake(brakePedalInput);
	} else {
		float brakePedalInput = 0; // Adjust as needed
		_ballistic->brake(brakePedalInput);
	}
	
	if(!anyDataCollectionMode){
		_ballistic->updateMotion(totalDelta);
	}
	
	// Get the car's position
	Vec3 carPosition = _ballistic->getPosition3D();
	
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
	Vec3 newPosition = carPosition + cameraOffset;
	
	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(carPosition);
	
	cursorDeltaX = 0;
	cursorDeltaY = 0;
	
}

void BallisticGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Ballistic");
	
	ImGui::End();
	
	
}
