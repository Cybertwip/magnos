#include "RoverGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "Car.h"
#include "Utils3d.h"
#include "Laser.h"


#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

float quaternionDot(const ax::Quaternion& q1, const ax::Quaternion& q2) {
	// Manually compute the dot product
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
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
	
	
	car = Car::create();
	this->addChild(car);
	
	auto director = Director::getInstance();
	
	director->setClearColor(Color4F(0.0f, 0.0f, 1.0f, 1.0f));
	
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
	
	return true;
	
}

void RoverGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	_defaultCamera->setPosition3D(Vec3(1.5f, 1.5f, -1.5f));
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
	if(car->isCalibrating()){
		return;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = true;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		steer = true;
		steerAngle = -6;
	}
	
	
	if(code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = true;
		steerAngle = 6;
	}
	
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = true;
	}
}

void RoverGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{

	if(car->isCalibrating()){
		return;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = false;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW || code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = false;
		steerAngle = 0;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = false;
	}
	
}

void RoverGameState::update(float) {
	float totalDelta = global_delta / 1000.0f;
	
	bool anyDataCollectionMode = car->isCollecting();
		
	if(accelerate || anyDataCollectionMode){
		
		car->accelerate(0.0f); // @TODO throttle
//
//		if(enable_lasers){
//			float laserVoltage = 5;
//			powerDraw = (laserVoltage / (float)gimbals.size()) * totalDelta;
//
//			totalPowerDrawn = 0;
//			for(auto gimbal : gimbals){
//				auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
//
//				totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
//			}
//
//			car->charge(totalPowerDrawn);
//
//		}
	} else {
		car->liftPedal();
	}

	
	car->update(totalDelta);
	if(car->anyLaserStatusOn() && enable_lasers){
		
//		float laserVoltage = 0;
//
//		for(auto laser : car->getLasers()){
//			laserVoltage += laser->getVoltageInput();
//		}
		
		//float powerDraw = (laserVoltage / (float)gimbals.size()) * totalDelta;
		
		//float totalPowerDrawn = 0;
//		for(auto gimbal : gimbals){
//			auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
//
//			totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
//		}
	}
	
	if(steer){
		car->steer(steerAngle);
	} else {
		car->steer(0);
	}
	
	if(brake){
		float brakePedalInput = 1.0f; // Adjust as needed
		car->brake(brakePedalInput);
	} else {
		float brakePedalInput = 0; // Adjust as needed
		car->brake(brakePedalInput);
	}
	
	if(!anyDataCollectionMode){
		car->updateMotion(totalDelta);
	}
	
	if(enable_lasers){
		for(auto laser : car->getLasers()){
			laser->update(totalDelta);
		}
	}
	
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
//
//	if(enable_lasers){
//		for(auto laser : car->getLasers()){
//			float storedPower = 0;
//			float powerToStore = (laser->getAccumulatedVoltage() / (float)gimbals.size()) * totalDelta;
//			for(auto gimbal : gimbals){
//				auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
//				storedPower += magnos->getCoilSystem().storePower(powerToStore);
//			}
//
//			if(storedPower != 0){
//				laser->dischargeAccumulatedVoltage(storedPower);
//			}
//
//		}
//
//	}
	
}

void RoverGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Engine");
	
	auto status = car->getEngine()->getMagnosFeedback().status;

	
	ImGui::Text("Status=%s", status.c_str());
	

	ImGui::Text("Input Voltage=%.4f", 1.5f);
	ImGui::Text("Peak Voltage=%.4f", car->getEngine()->getMagnosFeedback().peakEMF);
	
	static int desired_voltage = Settings::desired_target_voltage;
	//static int last_voltage_increase = desired_voltage;

	ImGui::Text("Target Voltage:%d", desired_voltage);
	
//
//	if(any_calibration || any_collection){
//		ImGui::BeginDisabled();
//		ImGui::SliderInt("Volts", &desired_voltage, min_voltage, max_voltage);
//		ImGui::EndDisabled();
//	} else {
//		ImGui::SliderInt("Volts", &desired_voltage, min_voltage, max_voltage);
//	}
	
//	if(last_voltage_increase != Settings::desired_target_voltage){
//		Settings::desired_target_voltage = last_voltage_increase;
//		for(auto gimbal : gimbals){
//			auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
//			magnos->getCoilSystem().setDesignedEMFPerSecond(Settings::desired_target_voltage / number_of_gimbals);
//			magnos->getCoilSystem().recalibrate();
//		}
//
//	}
	
	//last_voltage_increase = desired_voltage;
	
	ImGui::Text("Base Voltage=%.4f", car->getEngine()->getMagnosFeedback().baseEMF);
	ImGui::Text("Base + Gain Voltage=%.4f", car->getEngine()->getMagnosFeedback().EMF);
	//ImGui::Text("Recycled Filtered Voltage=%.4f", guiRecycledEMF); // @TODO maximize voltage
//
//	if(any_collection || any_calibration){
//		ImGui::BeginDisabled();
//		ImGui::Button("Collect Data");
//		ImGui::EndDisabled();
//	} else {
//		bool collectDataButtonPressed = false;
//
//		if (ImGui::Button("Collect Data")) {
//			collectDataButtonPressed = true;
//		}
//
//		if (collectDataButtonPressed) {
//			collectDataButtonPressed = false;
////
////			for(auto gimbal : gimbals){
////				auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
////
////				magnos->getCoilSystem().scheduleCollection();
////			}
//
//		}
//	}
	
	
	//ImGui::Text("Induced Current=%.4f", inducedCurrent);
	ImGui::End();
	
	ImGui::SetNextWindowPos(ImVec2(960, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Car");
	
	
	static float battery = 0;
	static float acceleration = 0;
	static float speed = 0;
	static float laser = 0;
	
	static float counter = 0;
	
	counter += fixed_delta;
	
	if(counter >= 1.0f){
		counter = 0;
		battery = 0;
		battery = car->getEngine()->getBatteryVoltage();
		acceleration = car->getAcceleration();
		speed = car->getSpeed();
		laser = 0;
		for(auto laserNode : car->getLasers()){
			laser += laserNode->getGuiMeasure();
		}
	}
	
	ImGui::Text("Battery Voltage=%.2f", battery);
	ImGui::Text("Accel m/s^2=%.2f", acceleration);
	ImGui::Text("Speed m/s=%.2f", speed);
	ImGui::Text("Laser v/s=%.2f", laser);
	ImGui::End();
	
}
