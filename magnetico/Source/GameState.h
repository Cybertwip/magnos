// GameState.h
#pragma once

#include <axmol.h>

class GameState : public ax::Node {
public:
	virtual ~GameState() {}
	
	virtual void setup(ax::Camera* defaultCamera) = 0;
	
	virtual void renderUI() = 0;
	
	virtual void updatePhysics(float dt) = 0;
	
	// mouse
	virtual void onMouseMove(ax::Event* event) = 0;
	
	// Keyboard
	virtual void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) = 0;
	virtual void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) = 0;
};
