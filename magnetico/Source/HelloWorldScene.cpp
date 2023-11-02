/****************************************************************************
 Copyright (c) 2017-2018 Xiamen Yaji Software Co., Ltd.
 Copyright (c) 2021 Bytedance Inc.

 https://axmolengine.github.io/

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/

#include "HelloWorldScene.h"
#include "RoverGameState.h"
#include "BasicCarGameState.h"
#include "WheelGameState.h"

#include "AdvancedCarGameState.h"
#include "RocketGameState.h"

#include "components/systems/CoilSystem.hpp"

#include "ImGui/ImGuiPresenter.h"

USING_NS_AX;
USING_NS_AX_EXT;

// Print useful error message instead of segfaulting when files are not there.
static void problemLoading(const char* filename)
{
    printf("Error while loading: %s\n", filename);
    printf(
        "Depending on how you compiled you might have to add 'Content/' in front of filenames in "
        "HelloWorldScene.cpp\n");
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
	if(!Scene::init()){
   	return false;
	}
	
	auto keyboardListener           = EventListenerKeyboard::create();
	keyboardListener->onKeyPressed  = AX_CALLBACK_2(HelloWorld::onKeyPressed, this);
	keyboardListener->onKeyReleased = AX_CALLBACK_2(HelloWorld::onKeyReleased, this);
	_eventDispatcher->addEventListenerWithFixedPriority(keyboardListener, 11);
	
	// Create a listener for mouse move events
	auto mouseListener = EventListenerMouse::create();
	
	// Set the callback function for mouse move events
	mouseListener->onMouseMove = AX_CALLBACK_1(HelloWorld::onMouseMove, this); // Assuming you're in the HelloWorld class
	
	// Register the mouse listener with the event dispatcher
	_eventDispatcher->addEventListenerWithSceneGraphPriority(mouseListener, this);
	
	// Enable mouse input (optional, if not already enabled)
	_director->getOpenGLView()->setCursorVisible(true);

	this->currentGameState = AdvancedCarGameState::create();
	
	this->addChild(currentGameState);

	currentGameState->setup(_defaultCamera);
	
	scheduleUpdate();

	return true;
}

void HelloWorld::onEnter(){
	Scene::onEnter();

	ImGuiPresenter::getInstance()->enableDPIScale(); // enable dpi scale for 4K display support, depends at least one valid ttf/ttc font was added.
	ImGuiPresenter::getInstance()->addRenderLoop("#im01", AX_CALLBACK_0(HelloWorld::onImGuiDraw, this), this);

}
void HelloWorld::onExit()
{
	Scene::onExit();
	ImGuiPresenter::getInstance()->removeRenderLoop("#im01");
}

void HelloWorld::onImGuiDraw()
{
#if !defined(__EMSCRIPTEN__)
	//ImGui::SetMouseCursor(ImGuiMouseCursor_None);
#endif
	
	ImGui::SetNextWindowPos(ImVec2(10, 550), ImGuiCond_FirstUseEver);
	ImGui::Begin("Sim Mode");
		
	
	// Button to switch to another state (replace with your other state's name)
	if (ImGui::Button("Basic"))
	{
		this->removeAllChildren();
		this->currentGameState = BasicCarGameState::create();
		this->currentGameState->setup(_defaultCamera);
		this->addChild(currentGameState);
	}
	
	// Button to switch to RoverGameState
	if (ImGui::Button("Car"))
	{
		this->removeAllChildren();
		this->currentGameState = AdvancedCarGameState::create();
		this->currentGameState->setup(_defaultCamera);
		this->addChild(currentGameState);
	}
	// Button to switch to RoverGameState
	if (ImGui::Button("Rover"))
	{
		this->removeAllChildren();
		this->currentGameState = RoverGameState::create();
		this->currentGameState->setup(_defaultCamera);
		this->addChild(currentGameState);
	}
	
	
	ImGui::End();

	this->currentGameState->renderUI();
}

void HelloWorld::onMouseMove(Event* event)
{
	this->currentGameState->onMouseMove(event);
}

void HelloWorld::onKeyPressed(EventKeyboard::KeyCode code, Event* event)
{
	this->currentGameState->onKeyPressed(code, event);
}

void HelloWorld::onKeyReleased(EventKeyboard::KeyCode code, Event* event)
{
	this->currentGameState->onKeyReleased(code, event);	
}

void HelloWorld::update(float delta)
{
	this->currentGameState->update(Settings::fixed_delta);
	
	int cycles_per_collection = Settings::physics_fixed_update / Settings::fps;
	
	int updates = 0;
	
	do{
		this->currentGameState->updatePhysics(Settings::fixed_physics_delta);
		updates++;
	} while(updates < cycles_per_collection);

}

