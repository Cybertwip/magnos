#include "RoverGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Car.h"
#include "Utils3d.h"

#include "ChVisualSystemAxmol.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

namespace{

// Current to Voltage conversion
float currentToVoltage(float current, float resistance = 4.0f) {
	return current * resistance;
}

float mpsToKmph(float speedMps) {
	return speedMps * 3.6;
}

void CreateTerrain(ChSystem& sys) {
	// Create the ground and obstacles
	auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
	auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
	ground->SetPos(ChVector<>(0, 0, -0.5));
	ground->SetBodyFixed(true);
	ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
	sys.Add(ground);
	
	// Create the first step of the stair-shaped obstacle
	auto mbox_1 = chrono_types::make_shared<ChBodyEasyBox>(2.4, 1.4, 0.1, 1000, true, true, ground_mat);
	mbox_1->SetPos(ChVector<>(3, 1, 0.05));
	mbox_1->SetBodyFixed(true);
	mbox_1->SetCollide(true);
	sys.Add(mbox_1);
	
	// Create the second step of the stair-shaped obstacle
	auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(1.6, 1.2, 0.2, 1000, true, true, ground_mat);
	mbox_2->SetPos(ChVector<>(3, 1, 0.1));
	mbox_2->SetBodyFixed(true);
	mbox_2->SetCollide(true);
	sys.Add(mbox_2);
	
	// Create the third step of the stair-shaped obstacle
	auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.8, 1.0, 0.3, 1000, true, true, ground_mat);
	mbox_3->SetPos(ChVector<>(3, 1, 0.15));
	mbox_3->SetBodyFixed(true);
	mbox_3->SetCollide(true);
	sys.Add(mbox_3);
}
}

USING_NS_AX;


RoverGameState::RoverGameState() {
	// Initialize private members and resources specific to the rover simulation
}

bool RoverGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init())
	{
		return false;
	}
	
	// Path to Chrono data files (textures, etc.)
	SetChronoDataPath(CHRONO_DATA_DIR);

	rover = std::make_unique<Curiosity>(&sys, chassis_type, wheel_type);
	
	sys.Set_G_acc(ChVector<>(0, 0, -9.81));
	
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
	
	// Create terrain and obstacles
	CreateTerrain(sys);

	// Create a Curiosity rover and the asociated driver
	////auto driver = chrono_types::make_shared<CuriositySpeedDriver>(1.0, 5.0);
	auto driver = chrono_types::make_shared<CuriosityDCMotorControl>();
	
	rover->SetDriver(driver);
	rover->Initialize(ChFrame<>(ChVector<>(0, 0, 0), QUNIT));

	std::cout << "Curiosity total mass: " << rover->GetRoverMass() << std::endl;
	std::cout << "  chassis:            " << rover->GetChassis()->GetBody()->GetMass() << std::endl;
	std::cout << "  wheel:              " << rover->GetWheel(CuriosityWheelID::C_LF)->GetBody()->GetMass() << std::endl;
	std::cout << std::endl;

	
	auto vis_axmol = chrono_types::make_shared<chrono::axmol::ChVisualSystemAxmol>();
	vis_axmol->AttachSystem(&sys);

	vis_axmol->Initialize(this);
	
	vis = vis_axmol;
	
	return true;
}

void RoverGameState::setup(ax::Camera* defaultCamera){
	
	// Create a directional light
	auto directionalLight = DirectionLight::create(Vec3(0, -1, -0.25f), Color3B::WHITE);
	this->addChild(directionalLight);
	auto directionalLight2 = DirectionLight::create(Vec3(0, 1, 0.25f), Color3B::WHITE);
	this->addChild(directionalLight2);

	// Create a point light
	auto pointLight = PointLight::create(Vec3(0, 5, 5), Color3B::WHITE, 100.0f);
	this->addChild(pointLight);
	
	// Create a spot light
	auto spotLight = SpotLight::create(Vec3(0, 5, 0), Vec3(0, -1, 0), Color3B::WHITE, 30, 45, 100.0f);
	this->addChild(spotLight);

	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	
	_defaultCamera->setPosition3D(Vec3(5, 5, 5));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0));
	
	
}

void RoverGameState::onMouseMove(Event* event)
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


void RoverGameState::onKeyPressed(EventKeyboard::KeyCode code, Event*)
{
	
}

void RoverGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{

	
}

void RoverGameState::update(float) {
	
	// Update Curiosity controls
	rover->Update();
	
	sys.DoStepDynamics(Settings::fixed_delta);

	
	
	// Get the car's position
	Vec3 carPosition = Vec3(0, 0, 0);
	
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

void RoverGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Engine");
	
	ImGui::End();
	
	ImGui::SetNextWindowPos(ImVec2(960, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Car");
	
	ImGui::End();
	
}
