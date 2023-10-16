#pragma once
#include "GameState.h"
#include "components/MovingAverageFilter.hpp"

#include "chrono_models/robot/curiosity/Curiosity.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
#include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::curiosity;

class Car;

class RoverGameState : public GameState {
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
	RoverGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void renderUI() override;

	CREATE_FUNC(RoverGameState);

	MovingAverageFilter inputAverageFilter = MovingAverageFilter(240);
	
	// Create a ChronoENGINE physical system
	ChSystemNSC sys;


};
