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

#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"

#ifdef CHRONO_IRRLICHT
#undef CHRONO_IRRLICHT
#endif

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
#include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif



#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
// specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif


#include "chrono_models/vehicle/artcar/ARTcar.h"

class StellaMagnosDriver;

using namespace chrono;
using namespace chrono::curiosity;


class AdvancedCarGameState : public GameState {
private:
	
	ax::Camera* _defaultCamera;
	
	std::string emf;
	std::string current;
	
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
	AdvancedCarGameState();
	
	virtual bool init() override;
	virtual void update(float delta) override;
	virtual void renderUI() override;

	CREATE_FUNC(AdvancedCarGameState);

	MovingAverageFilter inputAverageFilter = MovingAverageFilter(240);
	MovingAverageFilter outputAverageFilter = MovingAverageFilter(240);
	
	MovingAverageFilter engineAverageFilter = MovingAverageFilter(240);

	// The options are Scarecrow and FullRover
	CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;
	
	// Specify rover wheel type
	// The options are RealWheel, SimpleWheel, and CylWheel
	CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;
	// Create a ChronoENGINE physical system
	std::unique_ptr<vehicle::ChWheeledVehicle> vehicle;
	std::unique_ptr<vehicle::ChTerrain> terrain;
	std::shared_ptr<StellaMagnosDriver> driver;
	std::shared_ptr<ChVisualSystem> vis;
	
	float stallTorque = 300;

	ax::MeshRenderer* carMesh_;
};