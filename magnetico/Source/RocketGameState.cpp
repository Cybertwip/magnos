
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
	Rocket rocket(
				  total_rocket_mass, // mass
				  thrust_kN, // thrust
				  mass_RP1, // first_stage_mass
				  mass_LOX, // second_stage_mass
				  thrust_kN, // first_stage_thrust
				  thrust_kN * isp_multiplier, // second_stage_thrust
				  3000.0f * isp_multiplier, // specific_impulse
				  0.1f, // burn_rate
				  0.5f, // nozzle_radius
				  drag_coefficient, // drag_coefficient
				  M_PI * pow(rocket_radius, 2), // cross_sectional_area
				  total_propellant_mass, // initial_propellant_mass
				  max_valve_opening, // max_valve_opening
				  ax::Vec3(0.0f, r_initial + earth_radius_game, 0.0f), // initial_position
				  ax::Quaternion::identity() // initial_orientation
				  );
	
	
	return true;
	
}

void RocketGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
}

void RocketGameState::onMouseMove(Event* event)
{
	//EventMouse* e = static_cast<EventMouse*>(event);
	// Get the cursor delta since the last frame
}


void RocketGameState::onKeyPressed(EventKeyboard::KeyCode code, Event* event)
{
	
}

void RocketGameState::onKeyReleased(EventKeyboard::KeyCode code, Event* event)
{
	
	
}

void RocketGameState::update(float delta) {
	
}

void RocketGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Rocket");
	
	
	//ImGui::Text("Induced Current=%.4f", inducedCurrent);
	ImGui::End();
}
