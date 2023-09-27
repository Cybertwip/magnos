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
#include "MaritimeGimbal3D.h"

#include "ui/axmol-ui.h"

#include "ImGui/ImGuiPresenter.h"

#include <fstream>

#include <mlpack.hpp>
//#include <mlpack/methods/linear_regression/linear_regression.hpp>



USING_NS_AX;
USING_NS_AX_EXT;

float quaternionDot(const Quaternion& q1, const Quaternion& q2) {
	// Manually compute the dot product
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}


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
    //////////////////////////////
    // 1. super init first
    if (!Scene::init())
    {
        return false;
    }

	//this->initPhysicsWorld();
	//this->getPhysics3DWorld()->setGravity({});
	
	this->_defaultCamera->setNearPlane(0.01f);
	this->_defaultCamera->setFarPlane(10000);
	this->_defaultCamera->setFOV(60);
	this->_defaultCamera->setZoom(1);
	this->_defaultCamera->setPosition3D(Vec3(0.15f, 0.15f, -0.3f));
	this->_defaultCamera->setRotation3D(Vec3(0, 0, 0));
		
	auto magnosGimbal = MaritimeGimbal3D::create();
	gimbal = magnosGimbal;
	gimbal->setPosition3D(ax::Vec3(0, 0, 0));
	this->addChild(gimbal);
	
	magnosGimbal->attachPinball(this);
	
	//gimbal->addRodsToIronBall(dynamic_cast<IronBall*>(pinball), 0.0494f, 0.0005f); // Assuming a rod radius of 0.0005f


	this->_defaultCamera->lookAt(gimbal->getPosition3D());

	auto keyboardListener           = EventListenerKeyboard::create();
	keyboardListener->onKeyPressed  = AX_CALLBACK_2(HelloWorld::onKeyPressed, this);
	keyboardListener->onKeyReleased = AX_CALLBACK_2(HelloWorld::onKeyReleased, this);
	_eventDispatcher->addEventListenerWithFixedPriority(keyboardListener, 11);

    // scheduleUpdate() is required to ensure update(float) is called on every loop
    scheduleUpdate();

	auto director = Director::getInstance();

	director->setClearColor(Color4F(0.0f, 1.0f, 0.0f, 1.0f));

	return true;
}

void HelloWorld::onEnter(){
	Scene::onEnter();
	//ImGuiPresenter::getInstance()->addFont(R"(C:\Windows\Fonts\msyh.ttc)");
	/* For Simplified Chinese support, please use:
	 ImGuiPresenter::getInstance()->addFont(R"(C:\Windows\Fonts\msyh.ttc)", ImGuiPresenter::DEFAULT_FONT_SIZE,
	 ImGuiPresenter::CHS_GLYPH_RANGE::GENERAL);
	 */
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
	ImGui::Begin("window");
	
	auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
	
	float deltaTime = ImGui::GetIO().DeltaTime;
	static float deltaCounter = 0;
	static float guiCounter = 0;
	static float lastEMF = -1;
	static float lastMeasure = 0;
	static float guiBaseEMF = 0;
	static float guiEMF = 0;
	static float guiMeasure = 0;
	static float peakEMF = 0;
	static float baseAccumulatedEMF = 0;
	static float accumulatedEMF = 0;
	static float recycledEMF = 0;
	static float guiRecycledEMF = 0;

	deltaCounter += deltaTime;
	guiCounter += deltaTime;
	
	float inducedEMF = abs(magnos->getAlternatorSystem().emf);
	
	if(!magnos->getCoilSystem().calibrating() || magnos->getCoilSystem().adapting()){
		baseAccumulatedEMF = magnos->getCoilSystem().lastBaseAccumulatedEMF;
		accumulatedEMF = magnos->getCoilSystem().lastAccumulatedEMF;
		recycledEMF = magnos->getCoilSystem().lastRecycledEMF;
	}
	
	if(guiCounter >= 1){
		guiBaseEMF = baseAccumulatedEMF;
		guiEMF = accumulatedEMF;
		guiRecycledEMF = recycledEMF;
		accumulatedEMF = 0;
		guiCounter = 0;
	}
	
	if (guiEMF > peakEMF){
		if(!magnos->getCoilSystem().calibrating() || magnos->getCoilSystem().adapting()){
			peakEMF = guiEMF;
		} else {
			guiEMF = 0;
			guiMeasure = 0;
			peakEMF = 0;
		}
	}
	
	if(magnos->getCoilSystem().calibrating() && !magnos->getCoilSystem().adapting()){
		deltaCounter = 0;
		guiCounter = 0;
		lastEMF = -1;
		lastMeasure = 0;
		guiEMF = 0;
		guiMeasure = 0;
		peakEMF = 0;
		accumulatedEMF = 0;
		recycledEMF = 0;
		guiRecycledEMF = 0;
		baseAccumulatedEMF = 0;
	}
	
	if(magnos->getCoilSystem().calibrating()){
		ImGui::Text("Status=%s", "PID Calibration");
	} else if(magnos->getCoilSystem().adapting()){
		ImGui::Text("Status=%s", "Adaptive Calibration");
	} else {
		ImGui::Text("Status=%s", "Running");
	}
	ImGui::Text("Input Voltage=%.4f", 1.5f);
	ImGui::Text("Peak Voltage=%.4f", peakEMF);
	ImGui::Text("Target Voltage=%.4f", desired_voltage_increase_per_second);
	ImGui::Text("Base Voltage=%.4f", guiBaseEMF);
	ImGui::Text("Base + Gain Voltage=%.4f", guiEMF);
	ImGui::Text("Recycled Filtered Voltage=%.4f", guiRecycledEMF);

	//ImGui::Text("Induced Current=%.4f", inducedCurrent);
	ImGui::End();
}


void HelloWorld::onTouchesBegan(const std::vector<ax::Touch*>& touches, ax::Event* event)
{
    for (auto&& t : touches)
    {
        AXLOG("onTouchesBegan detected, X:%f  Y:%f", t->getLocation().x, t->getLocation().y);
    }
}

void HelloWorld::onTouchesMoved(const std::vector<ax::Touch*>& touches, ax::Event* event)
{
    for (auto&& t : touches)
    {
        AXLOG("onTouchesMoved detected, X:%f  Y:%f", t->getLocation().x, t->getLocation().y);
    }
}

void HelloWorld::onTouchesEnded(const std::vector<ax::Touch*>& touches, ax::Event* event)
{
    for (auto&& t : touches)
    {
        AXLOG("onTouchesEnded detected, X:%f  Y:%f", t->getLocation().x, t->getLocation().y);
    }
}

void HelloWorld::onMouseDown(Event* event)
{
    EventMouse* e = static_cast<EventMouse*>(event);
    AXLOG("onMouseDown detected, Key: %d", static_cast<int>(e->getMouseButton()));
}

void HelloWorld::onMouseUp(Event* event)
{
    EventMouse* e = static_cast<EventMouse*>(event);
    AXLOG("onMouseUp detected, Key: %d", static_cast<int>(e->getMouseButton()));
}

void HelloWorld::onMouseMove(Event* event)
{
    EventMouse* e = static_cast<EventMouse*>(event);
    AXLOG("onMouseMove detected, X:%f  Y:%f", e->getCursorX(), e->getCursorY());
}

void HelloWorld::onMouseScroll(Event* event)
{
    EventMouse* e = static_cast<EventMouse*>(event);
    AXLOG("onMouseScroll detected, X:%f  Y:%f", e->getScrollX(), e->getScrollY());
}

void HelloWorld::onKeyPressed(EventKeyboard::KeyCode code, Event* event)
{
	
}

void HelloWorld::onKeyReleased(EventKeyboard::KeyCode code, Event* event)
{
}

void HelloWorld::update(float delta)
{
    switch (_gameState)
    {
    case ExampleGameState::init:
    {
        _gameState = ExampleGameState::update;
        break;
    }

    case ExampleGameState::update:
    {
        /////////////////////////////
        // Add your codes below...like....
        // 
        // UpdateJoyStick();
        // UpdatePlayer();
        // UpdatePhysics();
        // ...
        break;
    }

    case ExampleGameState::pause:
    {
        /////////////////////////////
        // Add your codes below...like....
        //
        // anyPauseStuff()

        break;
    }

    case ExampleGameState::menu1:
    {    /////////////////////////////
        // Add your codes below...like....
        // 
        // UpdateMenu1();
        break;
    }

    case ExampleGameState::menu2:
    {    /////////////////////////////
        // Add your codes below...like....
        // 
        // UpdateMenu2();
        break;
    }

    case ExampleGameState::end:
    {    /////////////////////////////
        // Add your codes below...like....
        // 
        // CleanUpMyCrap();
        break;
    }

    } //switch
}

