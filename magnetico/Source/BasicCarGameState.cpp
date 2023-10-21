#include "BasicCarGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Car.h"
#include "Utils3d.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

namespace{
float quaternionDot(const ax::Quaternion& q1, const ax::Quaternion& q2) {
	// Manually compute the dot product
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

// Current to Voltage conversion
float currentToVoltage(float current, float resistance = 4.0f) {
	return current * resistance;
}

float voltsToCurrent(float voltage, float resistance) {
	if (resistance == 0.0f) {
		// Avoid division by zero
		return 0.0f;
	}
	
	return voltage / resistance;
}
float mpsToKmph(float speedMps) {
	return speedMps * 3.6;
}
}

USING_NS_AX;


BasicCarGameState::BasicCarGameState() {
	// Initialize private members and resources specific to the car simulation
}

bool BasicCarGameState::init() {
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

void BasicCarGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	_defaultCamera->setPosition3D(Vec3(1.5f, 1.5f, -1.5f));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0));
}

void BasicCarGameState::onMouseMove(Event* event)
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


void BasicCarGameState::onKeyPressed(EventKeyboard::KeyCode code, Event*)
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

void BasicCarGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
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

void BasicCarGameState::update(float) {
	float totalDelta = Settings::fixed_delta;
	
	bool anyDataCollectionMode = car->isCollecting();
		
	if(accelerate || anyDataCollectionMode){
		
		car->getEngine()->setEngineConsumption(400);
		
		car->accelerate(0.5f); // @TODO throttle

	} else {
		car->getEngine()->setEngineConsumption(0);

		car->liftPedal();
	}
	
	car->update(totalDelta);
	
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

void BasicCarGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Engine");
	
	auto status = car->getEngine()->getMagnosFeedback().status;

	ImGui::Text("Status=%s", status.c_str());
	ImGui::Text("Coil Voltage Draw=%.4f", 1.5f);
	ImGui::Text("Laser Voltage Draw=%.4f", 5.0f);
	ImGui::Text("Peak Voltage=%.4f", car->getEngine()->getMagnosFeedback().peakEMF);
	
	static int desired_voltage = EVEngine::max_voltage;
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
	
	float laserOutput = 0;
	float laserInput = 0;
	for(auto laserNode : car->getEngine()->getLasers()){
		laserOutput += laserNode->getGuiMeasure();
		laserInput += laserNode->getVoltageInput();
	}

	float coilInput = 0;
	for(auto magnos : car->getEngine()->getGimbals()){
		coilInput += currentToVoltage(magnos->getCoilSystem().current, Settings::circuit_resistance);
	}
	
	ImGui::Text("Input Voltage=%.2f",  car->isCalibrating() ? 0 : inputAverageFilter.filter(laserInput + coilInput));
	ImGui::Text("Base Voltage=%.4f", car->getEngine()->getMagnosFeedback().baseEMF);
	ImGui::Text("Base + Gain Voltage=%.4f", car->getEngine()->getMagnosFeedback().EMF + laserOutput);
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
	
	int cycles_per_collection = Settings::fixed_update / Settings::fps;

	counter += Settings::fixed_delta;
	
	if(counter >= 1.0f / (float)cycles_per_collection){
		counter = 0;
		battery = 0;
		battery = car->getEngine()->getBatteryVoltage();
		acceleration = car->getAcceleration();
		speed = car->getSpeed();
		laser = 0;
		for(auto laserNode : car->getEngine()->getLasers()){
			laser += laserNode->getGuiMeasure();
		}
	}
	
	ImGui::Text("Battery Voltage=%.2f", battery);
	ImGui::Text("Accel m/s^2=%.2f", acceleration);
	ImGui::Text("Speed km/h=%.2f", mpsToKmph(speed));
	ImGui::Text("Laser v/s=%.2f", laser);
	ImGui::End();
	
}
