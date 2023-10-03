#pragma once
#include "GameState.h"

class Car;

class RocketGameState : public GameState {
private:
	
	ax::Camera* _defaultCamera;
	
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

