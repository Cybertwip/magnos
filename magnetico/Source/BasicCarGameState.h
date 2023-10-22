#pragma once
#include "GameState.h"
#include "components/MovingAverageFilter.hpp"

class Car;

class BasicCarGameState : public GameState {
private:
	
	ax::Camera* _defaultCamera;
	
	std::string emf;
	std::string current;
	
	Car* car;
	
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
	
	
	void setup(ax::Camera* defaultCamera) override;
	
	// mouse
	void onMouseMove(ax::Event* event) override;

	// Keyboard
	void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) override;

public:
	BasicCarGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void renderUI() override;

	CREATE_FUNC(BasicCarGameState);


	MovingAverageFilter inputAverageFilter = MovingAverageFilter(240);
	MovingAverageFilter outputAverageFilter = MovingAverageFilter(240);
	
	MovingAverageFilter engineAverageFilter = MovingAverageFilter(240);


};
