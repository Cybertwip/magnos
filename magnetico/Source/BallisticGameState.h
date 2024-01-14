#pragma once
#include "GameState.h"
#include "components/MovingAverageFilter.hpp"

#include "extensions/Particle3D/PU/PUParticleSystem3D.h"

class BallisticGameState : public GameState {
private:
	
	ax::Camera* _defaultCamera;
	ax::MeshRenderer* _ballistic;
	
	float sensitivity = 0.01f;
	float cursorDeltaX = 0;
	float cursorDeltaY = 0;
	
	float cursorX = 0;
	float cursorY = 0;
	float prevCursorX = 0;
	float prevCursorY = 0;
	
	float horizontalAngle = 0.0f; // Initialize horizontal angle
	float verticalAngle = 0.0f;   // Initialize vertical angle
	
	
	bool brake = false;
	bool accelerate = false;
	bool steer = false;
	
	float steerAngle = 0;
	
	// Ballistic simulation parameters
	const float gravity = -9.8f;  // Acceleration due to gravity (adjust as needed)
	const float launchAngle = AX_DEGREES_TO_RADIANS(45.0f);  // Launch angle (adjust as needed)
	const float initialSpeed = 40.0f;  // Initial speed of the projectile (adjust as needed)
	
	ax::Vec3 ballisticPosition;
	ax::Vec3 ballisticVelocity;

	
	ax::PUParticleSystem3D* explosionParticle;

	void setup(ax::Camera* defaultCamera) override;
	
	// mouse
	void onMouseMove(ax::Event* event) override;

	// Keyboard
	void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) override;

public:
	BallisticGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void updatePhysics(float delta) override {}
	virtual void renderUI() override;

	CREATE_FUNC(BallisticGameState);

};
