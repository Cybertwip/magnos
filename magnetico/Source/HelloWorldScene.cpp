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
#include "Utils3d.h"

#include "ui/axmol-ui.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

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

MaritimeGimbal3D* createGimbal(ax::Node* parent, ax::Vec3 position){
	auto gimbal = MaritimeGimbal3D::create();
	gimbal->setPosition3D(position);
	parent->addChild(gimbal);
	gimbal->attachPinball();
	return gimbal;
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
	this->_defaultCamera->setFOV(90);
	this->_defaultCamera->setZoom(1);
	this->_defaultCamera->setPosition3D(Vec3(1.5f, 1.5f, -1.5f));
	this->_defaultCamera->setRotation3D(Vec3(0, 0, 0));
		
	auto gearBox = createCube(0.45f);
	auto gearBoxRenderer = ax::MeshRenderer::create();
	gearBoxRenderer->addMesh(gearBox);
	gearBoxRenderer->setPosition3D(ax::Vec3(0.65f, 0, 0));
	gearBoxRenderer->setRotation3D(Vec3(0, 180, 0));
	gearBoxRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	gearBoxRenderer->setTexture("kitty.jpg");
	gearBoxRenderer->setOpacity(50);
	gearBoxRenderer->setScaleX(0.75f);
	gearBoxRenderer->setScaleY(0.75f);
	gearBoxRenderer->setScaleZ(0.75f);
	
	
	gimbals.push_back(createGimbal(gearBoxRenderer, ax::Vec3(-0.25, 0, 0)));
	gimbals.push_back(createGimbal(gearBoxRenderer, ax::Vec3(0.25, 0, 0)));
	gimbals.push_back(createGimbal(gearBoxRenderer, ax::Vec3(0, 0, -0.25)));
	gimbals.push_back(createGimbal(gearBoxRenderer, ax::Vec3(0, 0, 0.25)));
	

	
	auto car = createCarWithWheels(1, 0.1, 0.2);
	this->addChild(gearBoxRenderer);
	
	this->addChild(car);

	for(auto gimbal : gimbals){
		auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);

		magnos->getCoilSystem().setDesignedEMFPerSecond(Settings::desired_target_voltage / number_of_gimbals);
	}

	this->_defaultCamera->lookAt(Vec3(0, 0, 0));

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
	
	
	float deltaTime = ImGui::GetIO().DeltaTime;
	static float guiCounter = 0;
	static float guiBaseEMF = 0;
	static float guiEMF = 0;
	static float peakEMF = 0;
	//static float recycledEMF = 0; // @TODO
	//static float guiRecycledEMF = 0;

	guiCounter += deltaTime;

	float baseAccumulatedEMF = 0;
	float accumulatedEMF = 0;

	bool any_calibration = false;

	for(auto gimbal : gimbals){
		auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);

		float inducedEMF = abs(magnos->getAlternatorSystem().emf);
		
		if(!magnos->getCoilSystem().calibrating()){
			baseAccumulatedEMF += magnos->getCoilSystem().lastBaseAccumulatedEMF;
			accumulatedEMF += magnos->getCoilSystem().lastAccumulatedEMF;
			//		recycledEMF = magnos->getCoilSystem().lastRecycledEMF;
		}

		if(magnos->getCoilSystem().calibrating()){
			any_calibration = true;
		}
	}

	
	if(guiCounter >= 1){
		guiBaseEMF = baseAccumulatedEMF;
		guiEMF = accumulatedEMF;
//		guiRecycledEMF = recycledEMF;
		accumulatedEMF = 0;
		guiCounter = 0;
	}
	
	if (guiEMF > peakEMF){
		if((!any_calibration) && !Settings::data_collection_mode){
			peakEMF = guiEMF;
		} else {
			guiEMF = 0;
			peakEMF = 0;
		}
	}
	
	if(any_calibration || Settings::data_collection_mode){
		guiCounter = 0;
		guiEMF = 0;
		peakEMF = 0;
		accumulatedEMF = 0;
//		recycledEMF = 0;
//		guiRecycledEMF = 0;
		baseAccumulatedEMF = 0;
	}
	
	if(any_calibration){
		ImGui::Text("Status=%s", "PID Calibration");
	} else {
		if(Settings::data_collection_mode){
			ImGui::Text("Status=%s", "Collecting Data");
		} else {
			ImGui::Text("Status=%s", "Running");
		}
	}
	ImGui::Text("Input Voltage=%.4f", 1.5f);
	ImGui::Text("Peak Voltage=%.4f", peakEMF);

	ImGui::Text("Target Voltage:");

	static int desired_voltage = Settings::desired_target_voltage;
	static int last_voltage_increase = desired_voltage;
		
	if(any_calibration || Settings::data_collection_mode){
		ImGui::BeginDisabled();
		ImGui::SliderInt("Volts", &desired_voltage, min_voltage, max_voltage);
		ImGui::EndDisabled();
	} else {
		ImGui::SliderInt("Volts", &desired_voltage, min_voltage, max_voltage);
	}
	
	if(last_voltage_increase != Settings::desired_target_voltage){
		Settings::desired_target_voltage = last_voltage_increase;
		for(auto gimbal : gimbals){
			auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
			magnos->getCoilSystem().setDesignedEMFPerSecond(Settings::desired_target_voltage / number_of_gimbals);
			magnos->getCoilSystem().recalibrate();
		}
		
	}
	   
   	last_voltage_increase = desired_voltage;

	ImGui::Text("Base Voltage=%.4f", guiBaseEMF);
	ImGui::Text("Base + Gain Voltage=%.4f", guiEMF);
	//ImGui::Text("Recycled Filtered Voltage=%.4f", guiRecycledEMF); // @TODO maximize voltage
	
	if(Settings::data_collection_mode || any_calibration){
		ImGui::BeginDisabled();
		ImGui::Button("Collect Data");
		ImGui::EndDisabled();
	} else {
		bool collectDataButtonPressed = false;
		
		if (ImGui::Button("Collect Data")) {
			collectDataButtonPressed = true;
		}
		
		if (collectDataButtonPressed) {
			collectDataButtonPressed = false;

			Settings::schedule_recalibration_for_collection = true;
			
		}
	}


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
	AXLOG("Mouse move detected, X:%f  Y:%f", e->getCursorX(), e->getCursorY());
	
	float sensitivity = 0.005f;

	// Get the cursor delta since the last frame
	float cursorDeltaX = e->getCursorX();
	float cursorDeltaY = e->getCursorY();
	
	// Calculate new camera rotation angles based on normalized cursor deltas
	float horizontalAngle = _defaultCamera->getRotation3D().y + cursorDeltaX;
	float verticalAngle = _defaultCamera->getRotation3D().x - cursorDeltaY;


	
	// Define the vertical angle constraints (adjust as needed)
	float minVerticalAngle = AX_DEGREES_TO_RADIANS(-10); // Minimum vertical angle (degrees)
	float maxVerticalAngle = AX_DEGREES_TO_RADIANS(45.0f);  // Maximum vertical angle (degrees)
	
	// Clamp the vertical angle within the specified range
	
	verticalAngle *= sensitivity;
	horizontalAngle *= sensitivity;

	verticalAngle = std::min(std::max(verticalAngle, minVerticalAngle), maxVerticalAngle);

	// Create quaternions for camera rotation
	Quaternion xRotation;
	xRotation.set(Vec3(1.0f, 0.0f, 0.0f), verticalAngle);
	Quaternion yRotation;
	yRotation.set(Vec3(0.0f, 1.0f, 0.0f), horizontalAngle);
	
	// Combine rotations to get the final orientation
	Quaternion newRotation = yRotation * xRotation;
	
	// Calculate the new camera position based on rotation angles
	float distanceFromCenter = (_defaultCamera->getPosition3D() - Vec3(0, 0, 0)).length();
	Vec3 newPosition = newRotation * Vec3(0, 0, -distanceFromCenter);
	
	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(Vec3(0, 0, 0));

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

