
#include "RocketGameState.h"


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
