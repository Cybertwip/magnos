#pragma once
#include "GameState.h"
#include "components/MovingAverageFilter.hpp"
#include "components/systems/CoilSystem.hpp"
#include "components/systems/MagnetSystem.hpp"

class Car;
class CoilSystem;
class MagnetSystem;
class MagneticBall;

class WheelGameState : public GameState {
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
	
	void updatePhysics(float delta) override {}
	
	// mouse
	void onMouseMove(ax::Event* event) override;

	// Keyboard
	void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) override;

	std::unique_ptr<CoilSystem> outerCoilSystem;
	std::unique_ptr<MagnetSystem> innerMagnetSystem;

	std::shared_ptr<msr::airlib::Node> outerNode;
	std::shared_ptr<msr::airlib::Node> innerNode;
	
	std::shared_ptr<MagneticBall> pinball;

public:
	WheelGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void renderUI() override;

	CREATE_FUNC(WheelGameState);

	MovingAverageFilter inputAverageFilter = MovingAverageFilter(240);

};
