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
#include "Car.h"
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
	
	car = Car::create();
	this->addChild(car);

	gimbals = car->getGimbals();
	
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

	auto plane = createPlane(1024, 1024, 1, 1);
	
	ax::MeshRenderer* planeRenderer = ax::MeshRenderer::create();
	planeRenderer->addMesh(plane);
	auto material = ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false);
	
	Texture2D::TexParams tRepeatParams;  // set texture parameters
	tRepeatParams.magFilter = tRepeatParams.minFilter = backend::SamplerFilter::NEAREST;
	tRepeatParams.sAddressMode                        = backend::SamplerAddressMode::REPEAT;
	tRepeatParams.tAddressMode                        = backend::SamplerAddressMode::REPEAT;

	auto checkerTexture = Director::getInstance()->getTextureCache()->addImage("checker.png");
	
	checkerTexture->setTexParameters(tRepeatParams);
	
	planeRenderer->setMaterial(material);
	planeRenderer->setTexture(checkerTexture);
	planeRenderer->setPositionY(-1.25f / 2 - 0.3f);
	this->addChild(planeRenderer);
	
	//@TODO Terrain
//	float terrainSize = 100.0f; // Adjust the size as needed
//	const std::string heightmapImage = "racing.png"; // Replace with your heightmap image file
//	const std::string texImage = "kitty.png"; // Replace with your heightmap image file
//	const std::string detailmapImage = "racing_detail.png"; // Replace with your heightmap image file
//
//	const std::string alphamapImage = "racing_alpha.png"; // Replace with your heightmap image file
//
//	ax::Terrain::DetailMap detailMap(detailmapImage);
//	ax::Terrain::TerrainData data(heightmapImage, texImage);
//
//	data._detailMapAmount = 1;
//	data._detailMaps[0]._detailMapSrc = detailmapImage;
//
//	data._alphaMapSrc = alphamapImage;
//
//	auto terrain = Terrain::create(data);
//
//	// Example positioning and adding terrain to the scene
//	terrain->setPosition3D(Vec3(0, 0, 0)); // Adjust the position as needed
//	this->addChild(terrain); // Add the terrain to your scene

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
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);

	ImGui::Begin("Engine");
	
	
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
	bool any_collection = false;

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
		
		if(magnos->getCoilSystem().collecting()){
			any_collection = true;
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
		if((!any_calibration) && !any_collection){
			peakEMF = guiEMF;
		} else {
			guiEMF = 0;
			peakEMF = 0;
		}
	}
	
	if(any_calibration || any_collection){
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
		if(any_collection){
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
		
	if(any_calibration || any_collection){
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
	
	if(any_collection || any_calibration){
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

			for(auto gimbal : gimbals){
				auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
				
				magnos->getCoilSystem().scheduleCollection();
			}
			
		}
	}


	//ImGui::Text("Induced Current=%.4f", inducedCurrent);
	ImGui::End();
	
	ImGui::SetNextWindowPos(ImVec2(960, 60), ImGuiCond_FirstUseEver);

	ImGui::Begin("Car");
	

	static float acceleration = 0;
	static float speed = 0;
	
	static float counter = 0;
	
	counter += deltaTime;
	
	if(counter >= 1.0f){
		counter = 0;
		acceleration = car->getAcceleration();
		speed = car->getSpeed();
	}

	ImGui::Text("Accel m/s^2=%.2f", acceleration);
	ImGui::Text("Speed m/s=%.2f", speed);
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
	// Get the cursor delta since the last frame
	
	prevCursorX = cursorX;
	prevCursorY = cursorY;
	
	cursorX = e->getDelta().x;
	cursorY = e->getDelta().y;
	
	cursorDeltaX = cursorX - prevCursorX;
	cursorDeltaY = cursorY - prevCursorY;
}

void HelloWorld::onMouseScroll(Event* event)
{
    EventMouse* e = static_cast<EventMouse*>(event);
    AXLOG("onMouseScroll detected, X:%f  Y:%f", e->getScrollX(), e->getScrollY());
}

void HelloWorld::onKeyPressed(EventKeyboard::KeyCode code, Event* event)
{
	for(auto gimbal : gimbals){
		auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
		
		if(magnos->getCoilSystem().calibrating()){
			return;
		}
	}
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		
		float powerDraw = 6 / gimbals.size();
		
		float totalPowerDrawn = 0;
		for(auto gimbal : gimbals){
			auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
			
			totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
		}
		

		car->accelerate(totalPowerDrawn);
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		car->steer(-6);
	}
	
	
	if(code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		car->steer(6);
	}

	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		float brakePedalInput = 1.0f; // Adjust as needed
		car->brake(brakePedalInput);
	}
}

void HelloWorld::onKeyReleased(EventKeyboard::KeyCode code, Event* event)
{
	for(auto gimbal : gimbals){
		auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
		
		if(magnos->getCoilSystem().calibrating()){
			return;
		}
	}

	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		
	}

	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW || code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		car->steer(0);
	}
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		float brakePedalInput = 0.0f; // Adjust as needed
		car->brake(brakePedalInput);
	}
	
}

void HelloWorld::update(float delta)
{
	for(auto gimbal : gimbals){
		gimbal->update(1.0f / 60.0f);
	}
	car->updateMotion(delta);
	
	// Get the car's position
	Vec3 carPosition = car->getPosition3D();

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

