#include "WheelGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"

#include "components/entities/Node.hpp"

#include "components/systems/MagnetSystem.hpp"
#include "components/systems/CoilSystem.hpp"

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

msr::airlib::Quaternionr createQuaternionFromAxisAngle(const msr::airlib::Vector3r& axis, float rotationAngle) {
	// Calculate the sine and cosine of half the angle
	float halfAngle = rotationAngle / 2.0f;
	float sinHalfAngle = sin(halfAngle);
	float cosHalfAngle = cos(halfAngle);
	
	// Return the quaternion
	return msr::airlib::Quaternionr(cosHalfAngle, axis.x() * sinHalfAngle, axis.y() * sinHalfAngle, axis.z() * sinHalfAngle);
}


// Helper function to convert degrees to radians
float degToRad(float degrees) {
	return degrees * (M_PI / 180.0f);
}

// Helper function to convert radians to degrees
float radToDeg(float radians) {
	return radians * (180.0f / M_PI);
}

msr::airlib::Vector3r directionFromAngle(float angleInDegrees) {
	float rad = degToRad(angleInDegrees);
	return msr::airlib::Vector3r(cos(rad), sin(rad), 0);
}


void applyTorqueAndRotate(std::shared_ptr<msr::airlib::Node> node, std::shared_ptr<MagneticBall> pinball, const msr::airlib::Vector3r& torque, float delta, const msr::airlib::Vector3r& axis) {
	auto magneticBall = pinball;
	// Project torque onto the Z-axis
	
	float length = sqrt(torque.x() * torque.x() + torque.y() * torque.y() + torque.z() * torque.z());
	
	float projectedTorqueMagnitude = length;
	
	// 1. Get the inertia from the magnetic ball
	float inertia = magneticBall->calculate_inertia();
	
	// 2. Calculate angular acceleration from torque and inertia
	float angularAcceleration = projectedTorqueMagnitude / inertia;
	
	// 3. Update angular speed based on this acceleration
	float oldAngularSpeed = node->getAngularSpeed();
	float newAngularSpeed = oldAngularSpeed + angularAcceleration * delta;
	
	float damping = 0.99f;
	newAngularSpeed *= damping;
	node->setAngularSpeed(newAngularSpeed);
	
	// 5. Calculate the rotation angle based on the updated angular speed
	float rotationAngle = newAngularSpeed * delta;
	
	// 6. Create the rotation and apply it (around Z-axis)
	msr::airlib::Quaternionr rotation
	= createQuaternionFromAxisAngle(axis, rotationAngle);
	msr::airlib::Quaternionr currentRotation = node->getRotationQuat();
	msr::airlib::Quaternionr newRotation = currentRotation * rotation;
	node->setRotationQuat(newRotation);
}


}

WheelGameState::WheelGameState() {
	// Initialize private members and resources specific to the car simulation
}

bool WheelGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init())
	{
		return false;
	}
	
	
	car = Car::create();
	this->addChild(car);
	
	auto director = ax::Director::getInstance();
	
	director->setClearColor(ax::Color4F(0.0f, 0.0f, 1.0f, 1.0f));
	
	auto plane = createPlane(1024, 1024, 1, 1);
	
	ax::MeshRenderer* planeRenderer = ax::MeshRenderer::create();
	planeRenderer->addMesh(plane);
	auto material = ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false);
	
	ax::Texture2D::TexParams tRepeatParams;  // set texture parameters
	tRepeatParams.magFilter = tRepeatParams.minFilter = ax::backend::SamplerFilter::NEAREST;
	tRepeatParams.sAddressMode                        = ax::backend::SamplerAddressMode::REPEAT;
	tRepeatParams.tAddressMode                        = ax::backend::SamplerAddressMode::REPEAT;
	
	auto checkerTexture = ax::Director::getInstance()->getTextureCache()->addImage("checker.png");
	
	checkerTexture->setTexParameters(tRepeatParams);
	
	planeRenderer->setMaterial(material);
	planeRenderer->setTexture(checkerTexture);
	planeRenderer->setPositionY(-1.25f / 2 - 0.3f);
	this->addChild(planeRenderer);
	
	outerCoilSystem =
	std::make_unique<CoilSystem>(
								 ax::FileUtils::getInstance()->getWritablePath(),
								 77,
								 80.0f,
								 5.0f, // resistance
								 1.0f, // current
								 700); // turns
	innerMagnetSystem = std::make_unique<MagnetSystem>();
	
	
	
	pinball = std::make_shared<IronBall>(0.05f);
	
	innerNode = std::make_shared<msr::airlib::Node>();
	outerNode = std::make_shared<msr::airlib::Node>();

	// Outer Node Magnets with NORTH polarity
	outerNode->addChild(outerCoilSystem->attach(0.105f + 0.0024f, MagnetDirection::NORTH, MagnetPolarity::NORTH));
	outerNode->addChild(outerCoilSystem->attach(0.105f + 0.0024f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH));
	outerNode->addChild(outerCoilSystem->attach(0.105f + 0.0024f, MagnetDirection::EAST, MagnetPolarity::NORTH));
	outerNode->addChild(outerCoilSystem->attach(0.105f + 0.0024f, MagnetDirection::WEST, MagnetPolarity::SOUTH));
	
	return true;
	
}

void WheelGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	_defaultCamera->setPosition3D(ax::Vec3(1.5f, 1.5f, -1.5f));
	_defaultCamera->setRotation3D(ax::Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(ax::Vec3(0, 0, 0));
}

void WheelGameState::onMouseMove(ax::Event* event)
{
	ax::EventMouse* e = static_cast<ax::EventMouse*>(event);
	// Get the cursor delta since the last frame
	
	prevCursorX = cursorX;
	prevCursorY = cursorY;
	
	cursorX = e->getDelta().x;
	cursorY = e->getDelta().y;
	
	cursorDeltaX = cursorX - prevCursorX;
	cursorDeltaY = cursorY - prevCursorY;
}

void WheelGameState::onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event*)
{
	
	if(code == ax::EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = true;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		steer = true;
		steerAngle = -6;
	}
	
	if(code ==  ax::EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = true;
		steerAngle = 6;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = true;
	}
}

void WheelGameState::onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event*)
{
	
	if(code == ax::EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = false;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_RIGHT_ARROW || code ==  ax::EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = false;
		steerAngle = 0;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = false;
	}
	
}

void WheelGameState::update(float) {
	float totalDelta = Settings::fixed_delta;
	
	bool anyDataCollectionMode = car->isCollecting();
		
	if(accelerate || anyDataCollectionMode){
		
		car->getEngine()->setEngineConsumption(400);
		
		car->accelerate(0.5f); // @TODO throttle

		outerCoilSystem->current = -43;

	} else {
		car->getEngine()->setEngineConsumption(0);

		car->liftPedal();
		
		outerCoilSystem->current = -0.0001;

	}
	
	//car->update(totalDelta);
	
	///////
	///
	///
	///
	///
	
	outerCoilSystem->update();
	innerMagnetSystem->update();
	
	int systemPolarity = outerCoilSystem->current >= 0 ? 1 : -1;

	auto wheel = car->rearLeftWheel;
	
	auto forces = outerCoilSystem->combineFieldsOrForces(msr::airlib::Vector3r(wheel->getWorldPosition3D().x, wheel->getWorldPosition3D().y, wheel->getWorldPosition3D().z));

	msr::airlib::Vector3r torqueInner = forces;
	
	applyTorqueAndRotate(innerNode, pinball, torqueInner, Settings::fixed_delta, msr::airlib::Vector3r(0, 0, 1) * systemPolarity);
	
	auto innerRotation = innerNode->getRotationQuat();
	
	car->rearSuspension->setRotationQuat(ax::Quaternion(innerRotation.x(), innerRotation.y(), innerRotation.z(), innerRotation.w()));

	wheel->setRotationQuat(ax::Quaternion(innerRotation.x(), innerRotation.y(), innerRotation.z(), innerRotation.w()));

	////////
	///
	///
	///
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
//		car->updateMotion(totalDelta);
	}
		
	// Get the car's position
	ax::Vec3 carPosition = wheel->getPosition3D();
	
	// Calculate new camera rotation angles based on normalized cursor deltas
	horizontalAngle += cursorDeltaX * sensitivity;
	verticalAngle -= cursorDeltaY * sensitivity;
	
	// Define the vertical angle constraints (adjust as needed)
	float minVerticalAngle = AX_DEGREES_TO_RADIANS(0); // Minimum vertical angle (degrees)
	float maxVerticalAngle = AX_DEGREES_TO_RADIANS(0); // Maximum vertical angle (degrees)
	
	// Clamp the vertical angle within the specified range
	verticalAngle = std::min(std::max(verticalAngle, minVerticalAngle), maxVerticalAngle);
	
	// Calculate the new camera position relative to the car
	float distanceFromCar = 1; // Adjust the distance as needed
	float cameraHeight = 0;   // Adjust the height as needed
	
	// Calculate the camera's offset from the car based on angles
	float horizontalOffset = distanceFromCar * sinf(horizontalAngle);
	float verticalOffset = distanceFromCar * cosf(horizontalAngle) * sinf(verticalAngle);
	float depthOffset = distanceFromCar * cosf(horizontalAngle) * cosf(verticalAngle);
	
	ax::Vec3 cameraOffset(horizontalOffset, cameraHeight + verticalOffset, depthOffset);
	
	// Calculate the new camera position
	ax::Vec3 newPosition = carPosition + cameraOffset;
	
	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(carPosition);
	
	cursorDeltaX = 0;
	cursorDeltaY = 0;
	
}

void WheelGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Engine");
	
	auto status = car->getEngine()->getMagnosFeedback().status;

	ImGui::Text("Status=%s", status.c_str());
	ImGui::Text("Coil Voltage Draw=%.4f", 1.5f);
	ImGui::Text("Lasear Voltage Draw=%.4f", 5.0f);
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
