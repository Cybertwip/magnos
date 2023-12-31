
#include "RocketGameState.h"
#include "Settings.h"
#include "Rocket.h"
#include "Utils3d.h"

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
	// Coordinates for Stockholm, Sweden
	float altitude_meters = 28.0;      // Altitude in meters above sea level (Stockholm
		
	float x = 0;
	float y = (earth_radius + altitude_meters);
	float z = 0;
	
	rocket = new Rocket(
						total_rocket_mass,  // mass
						thrust_kN,          // thrust
						mass_RP1,           // first_stage_mass
						mass_LOX,           // second_stage_mass
						3000.0f,             // specific_impulse
						FALCON_9_BURN_RATE, // burn_rate
						0.5f,                // nozzle_radius
						drag_coefficient,   // drag_coefficient
						M_PI * pow(rocket_radius, 2), // cross_sectional_area
						total_propellant_mass,        // initial_propellant_mass
						max_valve_opening,            // max_valve_opening
						ax::Vec3(x, y, z),            // initial_position in Earth-centered coordinates
						ax::Quaternion::identity()     // initial_orientation
						);
	
	rocket->setPosition3D(ax::Vec3(x, y, z));
	
	this->addChild(rocket);
	
		
	earthRenderer = ax::MeshRenderer::create("stella/earth.obj");
	earthRenderer->setPosition3D(ax::Vec3(0, 0, 0));
	earthRenderer->setScale(earth_radius * 2);
		
	this->addChild(earthRenderer);

	return true;
	
}

void RocketGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	_defaultCamera->setNearPlane(10);
	_defaultCamera->setFarPlane(1000000000);

	_defaultCamera->setPosition3D(rocket->getWorldPosition3D());
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
	rocket->update(delta);

	// Get the rocket's position
	Vec3 rocketPosition = rocket->getPosition3D();

	// Calculate new camera rotation angles based on normalized cursor deltas
	horizontalAngle += cursorDeltaX * delta;
	verticalAngle -= cursorDeltaY * delta;

	// Define the vertical angle constraints (adjust as needed)
	float minVerticalAngle = AX_DEGREES_TO_RADIANS(0); // Minimum vertical angle (degrees)
	float maxVerticalAngle = AX_DEGREES_TO_RADIANS(24); // Maximum vertical angle (degrees)
	
	// Clamp the vertical angle within the specified range
	verticalAngle = std::min(std::max(verticalAngle, minVerticalAngle), maxVerticalAngle);
	
	// Calculate the new camera position relative to the car
	float distance = 50; // Adjust the distance as needed
	float rocketOffset = 49.5f;   // Adjust the height as needed
	float cameraHeight = 0;
	
	Vec3 targetPosition = Vec3(rocketPosition.x, rocketPosition.y + rocketOffset / 2.0f, rocketPosition.z);

	horizontalAngle = std::roundf(horizontalAngle * 10.0f) / 10.0f;
	verticalAngle = std::roundf(verticalAngle * 10.0f) / 10.0f;

	// Calculate the camera's offset from the car based on angles
	float horizontalOffset = distance * sinf(horizontalAngle);
	float verticalOffset = (distance + cameraHeight) * cosf(horizontalAngle) * sinf(verticalAngle);
	float depthOffset = distance * cosf(horizontalAngle) * cosf(verticalAngle);
	
	Vec3 cameraOffset(horizontalOffset, verticalOffset, depthOffset);
	
	// Calculate the new camera position
	// Smoothly interpolate camera position
	Vec3 newPosition = targetPosition + cameraOffset;

	
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(targetPosition);
//	_defaultCamera->lookAt(Vec3());


	float altitude = rocket->get_altitude();
	
	uint8_t current_color[4];

	// Determine the current atmospheric layer
	if (altitude <= troposphere_distance) {
		float ratio = altitude / (troposphere_radius - earth_radius);
		lerp(troposphere_color, stratosphere_color, ratio, current_color);
	} else if (altitude <= stratosphere_distance) {
		float ratio = (altitude - (troposphere_radius - earth_radius)) / (stratosphere_radius - troposphere_radius);
		lerp(stratosphere_color, mesosphere_color, ratio, current_color);
	} else if (altitude <= mesosphere_distance) {
		float ratio = (altitude - (stratosphere_radius - earth_radius)) / (mesosphere_radius - stratosphere_radius);
		lerp(mesosphere_color, thermosphere_color, ratio, current_color);
	} else if (altitude <= thermosphere_distance) {
		float ratio = (altitude - (mesosphere_radius - earth_radius)) / (thermosphere_radius - mesosphere_radius);
		lerp(mesosphere_color, exosphere_color, ratio, current_color);
	} else {
		current_color[0] = exosphere_color[0];
		current_color[1] = exosphere_color[1];
		current_color[2] = exosphere_color[2];
		current_color[3] = exosphere_color[3];
	}

	ax::Director::getInstance()->setClearColor(ax::Color4F(current_color[0] / 255.0f, current_color[1] / 255.0f, current_color[2] / 255.0f, current_color[3] / 255.0f));

}

void RocketGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Rocket");
	
	float altitude = rocket->get_altitude();
	
	std::string layer = "Exosphere";
	
	// Determine the current atmospheric layer
	if (altitude <= troposphere_distance) {
		layer = "Troposphere";
	} else if (altitude <= stratosphere_distance) {
		layer = "Stratosphere";
	} else if (altitude <= mesosphere_distance) {
		layer = "Mesosphere";
	} else if (altitude <= thermosphere_distance) {
		layer = "Thermosphere";
	}

	ImGui::Text("Mass=%.4f", rocket->get_mass());
	ImGui::Text("Speed m/s=%.4f", rocket->get_linear_velocity());
	ImGui::Text("Altitude=%.4f", altitude);
	ImGui::Text("Layer=%s", layer.c_str());

	ImGui::End();
}
