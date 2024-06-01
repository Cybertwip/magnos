#pragma once
#include "GameState.h"
#include "components/MovingAverageFilter.hpp"

#include "extensions/Particle3D/PU/PUParticleSystem3D.h"

class NuclearParryGameState : public GameState {
private:
	
	// Define a struct to hold particle information
	struct Particle {
		ax::Vec3 position;
		bool activated;
	};

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
	
	
	// Movement booleans
	bool moveForward = false;
	bool moveBackward = false;
	bool moveLeft = false;
	bool moveRight = false;
	
	bool react = false;

	// Define particle simulation parameters
	int grid_size = 10;
	int activation_radius = 1;
	int initial_particles = 2;
	
	// Initialize particle states
	std::vector<Particle> activeParticles;

	std::vector<ax::MeshRenderer*> particleRenderers;
	
	void initializeParticles();
	
	void setup(ax::Camera* defaultCamera) override;

	// mouse
	void onMouseMove(ax::Event* event) override;
	
	// Keyboard
	void onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	void onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event* event) override;
	

public:
	NuclearParryGameState();
	
	CREATE_FUNC(NuclearParryGameState);

	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void updatePhysics(float) override {}
	virtual void renderUI() override;

};
