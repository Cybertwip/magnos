#pragma once
#include "GameState.h"

class Rocket;

class RocketGameState : public GameState {
private:
	Rocket* rocket;
	ax::Camera* _defaultCamera;
	
	float sensitivity = 0.01f;
	float cursorDeltaX = 0;
	float cursorDeltaY = 0;
	
	float cursorX = 0;
	float cursorY = 0;
	float prevCursorX = 0;
	float prevCursorY = 0;
	
	float horizontalAngle = 0.0f; // Initialize horizontal angle
	float verticalAngle = 0.0f;   // Initialize vertical angle

	
	void setup(ax::Camera* defaultCamera) override;
	
	// mouse
	void onMouseMove(ax::Event* event) override;
	
	// Keyboard
	void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	
public:
	RocketGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void renderUI() override;
	
	CREATE_FUNC(RocketGameState);
	
};

