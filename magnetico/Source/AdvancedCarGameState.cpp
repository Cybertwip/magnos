#include "AdvancedCarGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Car.h"
#include "Utils3d.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "ChVisualSystemAxmol.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"

#include "components/EVEngine.hpp"
#include "components/Magnos.hpp"
#include "components/Battery.hpp"
#include "components/Laser.hpp"

#include <memory>

namespace{

// Current to Voltage conversion
float currentToVoltage(float current, float resistance = 4.0f) {
	return current * resistance;
}

float mpsToKmph(float speedMps) {
	return speedMps * 3.6;
}

}

class WheelMotor {
public:
	WheelMotor(float maxVoltage, float efficiency)
	: maxVoltage_(maxVoltage), efficiency_(efficiency), voltage_(0.0f), currentConsumption_(0.0f) {}
	
	void SetVoltage(float voltage) {
		if (voltage < 0.0f || voltage > maxVoltage_) {
			// Handle invalid voltage input
			// You can throw an exception, log an error, or take appropriate action here.
			return;
		}
		voltage_ = voltage;
		currentConsumption_ = voltage / efficiency_;
	}
	
	float GetCurrentConsumption() const {
		return currentConsumption_;
	}
	
	float GetMaxVoltage() const {
		return maxVoltage_;
	}
	
private:
	float maxVoltage_;
	float efficiency_;
	float voltage_;
	float currentConsumption_;
};

class StellaMagnosDriver : public CuriosityDriver {
public:
	StellaMagnosDriver(Curiosity& vehicle, msr::airlib::Vector3r position);
	
	~StellaMagnosDriver() {}
	
	void accelerate(float voltage);
	
	std::shared_ptr<EVEngine> getEngine() const {
		return engine_;
	}
	
private:
	virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::TORQUE; }
	virtual void Update(double time) override;
	
	double m_ramp;
	double m_speed;
	
	std::shared_ptr<EVEngine> engine_;
	Curiosity& rover;
	
	std::vector<WheelMotor> wheelMotors_;
};

StellaMagnosDriver::StellaMagnosDriver(Curiosity& vehicle, msr::airlib::Vector3r position) : rover(vehicle) {
	engine_ = std::make_shared<EVEngine>(120);
	engine_->setPosition3D(position);
	engine_->init();
	
	for(int i = 0; i<6; ++i){
		wheelMotors_.push_back(WheelMotor(10, 0.5f));
	}
}

void StellaMagnosDriver::accelerate(float throttle){
	for(auto& motor : wheelMotors_){
		motor.SetVoltage(motor.GetMaxVoltage() * throttle);
	}
}

void StellaMagnosDriver::Update(double time) {
	auto roverPosition = rover.GetChassisPos();
	msr::airlib::Vector3r position = msr::airlib::Vector3r(roverPosition.x(), roverPosition.y(), roverPosition.z());
	
	engine_->setPosition3D(position);
	

	float consumption = 0.0f;
	std::vector<float> angularSpeeds;
	
	for (auto& motor : wheelMotors_) {
		consumption += motor.GetCurrentConsumption();
		
		// Assuming a linear relationship between current consumption and angular speed
		// Replace this with the actual relationship from your rover's specifications.
		float angularSpeed = motor.GetCurrentConsumption() * 16;
		angularSpeeds.push_back(angularSpeed);
	}
	
	engine_->setEngineConsumption(consumption);
	
	// Use angular speeds to set drive_speeds (assuming 6-wheel configuration).
	for(int i = 0; i<angularSpeeds.size(); ++i){
		curiosity->GetDriveshaft(static_cast<CuriosityWheelID>(i))->SetAppliedTorque(-angularSpeeds[i]);

	}
}
#include "chrono_models/vehicle/duro/Duro_Vehicle.h"

USING_NS_AX;


// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON files for engine and transmission
std::string engine_file("hmmwv/powertrain/HMMWV_EngineSimple.json");
std::string transmission_file("hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json");

// JSON files tire models
std::string tmeasy_tire_file("hmmwv/tire/HMMWV_TMeasy_converted.json");
std::string fiala_tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(32, 0, 0.6);

// Logging of seat acceleration data on flat road surface is useless and would lead to distorted results
double xend = 400.0;  // end logging here, this also the end of our world

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

AdvancedCarGameState::AdvancedCarGameState() {
	// Initialize private members and resources specific to the rover simulation
}

bool AdvancedCarGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init())
	{
		return false;
	}
	
	// Path to Chrono data files (textures, etc.)
	SetChronoDataPath(CHRONO_DATA_DIR);

	std::string vehicleDataPath =
	std::string{ CHRONO_DATA_DIR } + "vehicle/";
	
	vehicle::SetDataPath(vehicleDataPath.c_str());
	int iTire = 2;
	
	double target_speed = 420.0;
	
	// JSON files for terrain
	std::string rigidterrain_file("terrain/RigidObstacle1.json");
	
	GetLog() << "\n"
	<< "Speed       = " << target_speed << " m/s\n"
	<< "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka89, 4=Pacejka02, 5=Rigid) = " << iTire << "\n";

	
	// --------------------------
	// Create the various modules
	// --------------------------
	ChContactMethod contact_method = ChContactMethod::SMC;
//	vehicle = std::make_unique<duro::Duro_Vehicle>(false,
//		BrakeType::SHAFTS,
//	 	SteeringTypeWV::PITMAN_ARM_SHAFTS,
//		contact_method,
//		CollisionType::PRIMITIVES);
	
	vehicle = std::make_unique<WheeledVehicle>(vehicle::GetDataFile(vehicle_file), contact_method);


	vehicle->Initialize(ChCoordsys<>(initLoc, QUNIT));
	vehicle->GetChassis()->SetFixed(false);
	vehicle->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
	vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	vehicle->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
	
//	terrain = std::make_unique<RigidTerrain>(vehicle->GetSystem(), vehicle::GetDataFile(rigidterrain_file));

//	if (iTire == 5) {
//		ChContactMaterialData minfo;
//		minfo.mu = 0.9f;
//		minfo.cr = 0.01f;
//		minfo.Y = 2e7f;
//		auto terrain_mat = minfo.CreateMaterial(contact_method);
//		terrain->EnableCollisionMesh(terrain_mat, std::abs(initLoc.x()) + 5);
//	}
	
	auto terrainImpl = std::make_unique<RandomSurfaceTerrain>(vehicle->GetSystem(), xend);
	
	terrainImpl->Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_A_CORR, vehicle->GetWheeltrack(0));

	terrain = std::move(terrainImpl);

	
	
	// Create and initialize the powertrain system
	auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
	auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
	auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
	vehicle->InitializePowertrain(powertrain);
	
	// Create and initialize the tires
	for (auto& axle : vehicle->GetAxles()) {
		switch (iTire) {
			default:
			case 1: {
				auto tireL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
				auto tireR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
				vehicle->InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
				vehicle->InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
				break;
			}
			case 2: {
				auto tireL = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
				auto tireR = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
				vehicle->InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
				vehicle->InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
				break;
			}
			case 3: {
				auto tireL = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
				auto tireR = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
				vehicle->InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
				vehicle->InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
				break;
			}
			case 5: {
				auto tireL = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
				auto tireR = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
				vehicle->InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
				vehicle->InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
				
				break;
			}
		}
	}

	// Create the driver
	auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
	driver = std::make_shared<ChPathFollowerDriver>(*vehicle, vehicle::GetDataFile(steering_controller_file),
								vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
	driver->Initialize();
	
	auto vis_axmol = chrono_types::make_shared<chrono::axmol::ChVisualSystemAxmol>();
	vis_axmol->AttachSystem(vehicle->GetSystem());
	
	vis_axmol->Initialize(this);
	
	vis = vis_axmol;
	
	carMesh_ = ax::MeshRenderer::create("polestar/Polestar1_Final01.obj");
		
	this->addChild(carMesh_);
	
	auto vehiclePosition = vehicle->GetChassis()->GetPos();
	auto rotation = vehicle->GetChassis()->GetRot();
	
	ax::Quaternion quaternion = ax::Quaternion(rotation.e1(),
					   rotation.e2(),
					   rotation.e3(),
					   rotation.e0());
	
	carMesh_->setPosition3D(Vec3(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z()));

	carMesh_->setRotationQuat(quaternion);

	return true;
}

void AdvancedCarGameState::setup(ax::Camera* defaultCamera){
	
	// Create a directional light
	auto directionalLight = DirectionLight::create(Vec3(0, 0, -1), Color3B::WHITE);
	this->addChild(directionalLight);
	auto directionalLight2 = DirectionLight::create(Vec3(0, 0, 1), Color3B::WHITE);
	
	this->addChild(directionalLight2);

	// Create a point light
	auto pointLight = PointLight::create(Vec3(0, 5, 5), Color3B::WHITE, 1000.0f);
	this->addChild(pointLight);
	
	// Create a spot light
	auto spotLight = SpotLight::create(Vec3(0, 0, 0), Vec3(0, 0, 1), Color3B::WHITE, 30, 45, 1000.0f);

	carMesh_->addChild(spotLight);

	auto skybox = ax::Skybox::create("white.jpg", "white.jpg", "white.jpg", "white.jpg", "white.jpg", "white.jpg");
	
	this->addChild(skybox);
	
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	
	_defaultCamera->setPosition3D(Vec3(5, 5, 5));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0), Vec3(0, 0, 1));
	
	
}

void AdvancedCarGameState::onMouseMove(Event* event)
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


void AdvancedCarGameState::onKeyPressed(EventKeyboard::KeyCode code, Event*)
{
//	if(driver->getEngine()->isCalibrating()){
//		return;
//	}

	if(code == EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = true;
	}
	
	if(code == EventKeyboard::KeyCode::KEY_RIGHT_ARROW){
		steer = true;
		steerAngle = 6;
	}
	
	
	if(code ==  EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = true;
		steerAngle = -6;
	}
	
	
	if(code == EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = true;
	}
	
}

void AdvancedCarGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{

//	if(driver->getEngine()->isCalibrating()){
//		return;
//	}
//
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

void AdvancedCarGameState::update(float delta) {
	// Get driver inputs
	DriverInputs driver_inputs = driver->GetInputs();
	// Update modules (process inputs from other modules)
	
	static long time = 0;
	time += Settings::global_delta;
	
	driver->Synchronize(time);
	vehicle->Synchronize(time, driver_inputs, *terrain);
	terrain->Synchronize(time);
//	vis->Synchronize(time, driver_inputs);
	
	// Advance simulation for one timestep for all modules
	driver->Advance(delta);
	vehicle->Advance(delta);
	terrain->Advance(delta);
//	vis->Advance(step_size);
	
	
	// Update Curiosity controls
//
//	driver->getEngine()->update(Settings::fixed_delta);
//
//	if(accelerate){
//		stallTorque += 1200 * Settings::fixed_delta;
//		driver->accelerate(0.5f);
//	} else {
//		driver->accelerate(0.0f);
//	}
//
//	rover->Update();
//
//	sys.DoStepDynamics(Settings::fixed_delta);
//
//	if(steer){
//		driver->SetSteering(steerAngle);
//	} else {
//		driver->SetSteering(0);
//	}
//
//
	// Get the car's position
	auto vehiclePosition = vehicle->GetPos();
	auto chassisPosition = vehicle->GetChassis()->GetPos();
	auto rotation = vehicle->GetChassis()->GetRot();
	
	carMesh_->setScale(1.215f);
	carMesh_->setScaleZ(1.200f);
	carMesh_->setPosition3D(Vec3(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z()));
	
	// Original quaternion from rotation.e1(), e2(), e3(), e0()
	ax::Quaternion originalQuaternion = ax::Quaternion(rotation.e1(), rotation.e2(), rotation.e3(), rotation.e0());
	
	// Z-Up rotation quaternion (90 degrees around the X-axis)
	float zUpAngle = M_PI_2; // 90 degrees in radians
	ax::Quaternion zUpRotation = ax::Quaternion(sin(zUpAngle / 2), 0, 0, cos(zUpAngle / 2));
	
	// Yaw rotation quaternion (45 degrees around the Z-axis)
	float yawAngle = M_PI / 2; // 45 degrees in radians
	ax::Quaternion yawRotation = ax::Quaternion(0, 0, sin(yawAngle / 2), cos(yawAngle / 2));
	
	// Multiply the original quaternion by the Z-Up and Yaw rotations
	ax::Quaternion finalQuaternion = yawRotation * zUpRotation * originalQuaternion;
	
	// Set the final quaternion for the carMesh_
	carMesh_->setRotationQuat(finalQuaternion);

	Vec3 carPosition = Vec3(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());

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

	Vec3 cameraOffset(horizontalOffset, depthOffset, cameraHeight + verticalOffset);

	// Calculate the new camera position
	Vec3 newPosition = carPosition + cameraOffset;

	// Set the camera's new position and look-at point
	_defaultCamera->setPosition3D(newPosition);
	_defaultCamera->lookAt(carPosition, Vec3(0, 0, 1));

	cursorDeltaX = 0;
	cursorDeltaY = 0;
}

void AdvancedCarGameState::renderUI() {
//	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
//
//
//	float laserOutput = 0;
//	float laserInput = 0;
//	for(auto laserNode : driver->getEngine()->getLasers()){
//		laserOutput += laserNode->getGuiMeasure();
//		laserInput += laserNode->getVoltageInput();
//	}
//
//	float coilInput = 0;
//	for(auto magnos : driver->getEngine()->getGimbals()){
//		coilInput += currentToVoltage(magnos->getCoilSystem().current, Settings::circuit_resistance);
//	}
//
//	ImGui::Begin("Engine");
//
//	auto status = driver->getEngine()->getMagnosFeedback().status;
//
//	ImGui::Text("Status=%s", status.c_str());
//
//	ImGui::Text("Consumption (VDC)=%.2f",  driver->getEngine()->isCalibrating() ? 0 : inputAverageFilter.filter( driver->getEngine()->getEngineConsumption()));
//
//	ImGui::End();
//
//	ImGui::SetNextWindowPos(ImVec2(960, 60), ImGuiCond_FirstUseEver);
//
//	ImGui::Begin("Rover");
//
//
//	static float battery = 0;
//	static float acceleration = 0;
//	static float speed = 0;
//	static float laser = 0;
//
//	static float counter = 0;
//
//	int cycles_per_collection = Settings::fixed_update / Settings::fps;
//
//	counter += Settings::fixed_delta;
//
//	if(counter >= 1.0f / (float)cycles_per_collection){
//		counter = 0;
//		battery = 0;
//		battery = driver->getEngine()->getBatteryVoltage();
//		float linearAvg = 0.0f;
//		linearAvg += rover->GetChassisVel().x();
//		linearAvg += rover->GetChassisVel().y();
//		linearAvg += rover->GetChassisVel().z();
//
//		linearAvg /= 3.0f;
//
////		acceleration = car->getAcceleration();
//		speed = linearAvg;
//	}
//
//	ImGui::Text("Battery Voltage=%.2f", battery);
////	ImGui::Text("Accel m/s^2=%.2f", acceleration);
//	ImGui::Text("Speed km/h=%.2f", mpsToKmph(speed));
//	ImGui::End();
//

}
