
#include "RocketGameState.h"
#include "Settings.h"
#include "Rocket.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"


USING_NS_AX;


RocketGameState::RocketGameState() {
	// Initialize private members and resources specific to the rover simulation
}

bool RocketGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init())
	{
		return false;
	}
	// Instantiate a Rocket object with the given parameters
	rocket = new Rocket(
				  total_rocket_mass, // mass
				  thrust_kN, // thrust
				  mass_RP1, // first_stage_mass
				  mass_LOX, // second_stage_mass
				  thrust_kN, // first_stage_thrust
				  thrust_kN, // second_stage_thrust
				  3000.0f, // specific_impulse
				  FALCON_9_BURN_RATE, // burn_rate
				  0.5f, // nozzle_radius
				  drag_coefficient, // drag_coefficient
				  M_PI * pow(rocket_radius, 2), // cross_sectional_area
				  total_propellant_mass, // initial_propellant_mass
				  max_valve_opening, // max_valve_opening
				  ax::Vec3(0.0f, r_initial + earth_radius_game, 0.0f), // initial_position
				  ax::Quaternion::identity() // initial_orientation
				  );
	
	
	rocket->setPosition3D(ax::Vec3(0, 0, 0));
	
	this->addChild(rocket);
	return true;
	
}

void RocketGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setPosition3D(Vec3(10, 10, 10));
}

void RocketGameState::onMouseMove(Event* event)
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


void RocketGameState::onKeyPressed(EventKeyboard::KeyCode code, Event* event)
{
	
}

void RocketGameState::onKeyReleased(EventKeyboard::KeyCode code, Event* event)
{
	
	
}

void RocketGameState::update(float delta) {

	// Get the rocket's position
	Vec3 targetPosition = rocket->getPosition3D();
	
	// Calculate new camera rotation angles based on normalized cursor deltas
	horizontalAngle += cursorDeltaX * sensitivity;
	verticalAngle -= cursorDeltaY * sensitivity;

	// Define the vertical angle constraints (adjust as needed)
	float minVerticalAngle = AX_DEGREES_TO_RADIANS(0); // Minimum vertical angle (degrees)
	float maxVerticalAngle = AX_DEGREES_TO_RADIANS(60.0f); // Maximum vertical angle (degrees)
	
	// Clamp the vertical angle within the specified range
	verticalAngle = std::min(std::max(verticalAngle, minVerticalAngle), maxVerticalAngle);
	
	// Calculate the new camera position relative to the car
	float distance = 100.0f; // Adjust the distance as needed
	float cameraHeight = 20.0f;   // Adjust the height as needed
	
	// Calculate the camera's offset from the car based on angles
	float horizontalOffset = distance * sinf(horizontalAngle);
	float verticalOffset = distance * cosf(horizontalAngle) * sinf(verticalAngle);
	float depthOffset = distance * cosf(horizontalAngle) * cosf(verticalAngle);
	
	Vec3 cameraOffset(horizontalOffset, cameraHeight + verticalOffset, depthOffset);
	
	// Calculate the new camera position
	Vec3 newPosition = targetPosition + cameraOffset;
	
	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(targetPosition);

}

void RocketGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Rocket");
	
	
	ImGui::Text("Mass=%.4f", rocket->get_mass());
	ImGui::Text("Altitude=%.4f", rocket->get_altitude());

	ImGui::End();
}
