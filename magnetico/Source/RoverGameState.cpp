#include "RoverGameState.h"

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

void CreateTerrain(ChSystem& sys) {
	// Create the ground and obstacles
	auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
	auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
	ground->SetPos(ChVector<>(0, 0, -0.5));
	ground->SetBodyFixed(true);
	ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
	sys.Add(ground);
	
	return;
	
	// Create the first step of the stair-shaped obstacle
	auto mbox_1 = chrono_types::make_shared<ChBodyEasyBox>(2.4, 1.4, 0.1, 1000, true, true, ground_mat);
	mbox_1->SetPos(ChVector<>(3, 1, 0.05));
	mbox_1->SetBodyFixed(true);
	mbox_1->SetCollide(true);
	sys.Add(mbox_1);
	
	// Create the second step of the stair-shaped obstacle
	auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(1.6, 1.2, 0.2, 1000, true, true, ground_mat);
	mbox_2->SetPos(ChVector<>(3, 1, 0.1));
	mbox_2->SetBodyFixed(true);
	mbox_2->SetCollide(true);
	sys.Add(mbox_2);
	
	// Create the third step of the stair-shaped obstacle
	auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.8, 1.0, 0.3, 1000, true, true, ground_mat);
	mbox_3->SetPos(ChVector<>(3, 1, 0.15));
	mbox_3->SetBodyFixed(true);
	mbox_3->SetCollide(true);
	sys.Add(mbox_3);

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

class CuriosityMagnosDriver : public CuriosityDriver {
public:
	CuriosityMagnosDriver(Curiosity& vehicle, msr::airlib::Vector3r position);
	
	~CuriosityMagnosDriver() {}
	
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

CuriosityMagnosDriver::CuriosityMagnosDriver(Curiosity& vehicle, msr::airlib::Vector3r position) : rover(vehicle) {
	engine_ = std::make_shared<EVEngine>(120);
	engine_->setPosition3D(position);
	engine_->init();
	
	for(int i = 0; i<6; ++i){
		wheelMotors_.push_back(WheelMotor(10, 0.5f));
	}
}

void CuriosityMagnosDriver::accelerate(float throttle){
	for(auto& motor : wheelMotors_){
		motor.SetVoltage(motor.GetMaxVoltage() * throttle);
	}
}

void CuriosityMagnosDriver::Update(double time) {
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
	
	// Path to Chrono data files (textures, etc.)
	SetChronoDataPath(CHRONO_DATA_DIR);

	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

	sys.Set_G_acc(ChVector<>(0, 0, -9.81));

	rover = std::make_unique<Curiosity>(&sys, chassis_type, wheel_type);
	
	// Global parameter for moving patch size:
	double wheel_range = 0.5;

//	// Create obstacles
//	std::vector<std::shared_ptr<ChBodyAuxRef>> rock;
//	std::vector<std::string> rock_meshfile = {
//		"robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
//		"robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
//		"robot/curiosity/rocks/rock3.obj", "robot/curiosity/rocks/rock3.obj"   //
//	};
//	std::vector<ChVector<>> rock_pos = {
//		ChVector<>(-2.5, -0.3, -1.0), ChVector<>(-2.5, -0.3, +1.0), //
//		ChVector<>(-1.0, -0.3, -1.0), ChVector<>(-1.0, -0.3, +1.0), //
//		ChVector<>(+0.5, -0.3, -1.0), ChVector<>(+0.5, -0.3, +1.0) //
//	};
//	std::vector<double> rock_scale = {
//		0.8,  0.8,   //
//		0.45, 0.45,  //
//		0.45, 0.45   //
//	};
//	double rock_density = 8000;
//	std::shared_ptr<ChMaterialSurface> rock_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());
//
//	for (int i = 0; i < 6; i++) {
//		auto mesh = chrono::geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), false, true);
//		mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(rock_scale[i]));
//
//		double mass;
//		ChVector<> cog;
//		ChMatrix33<> inertia;
//		mesh->ComputeMassProperties(true, mass, cog, inertia);
//		ChMatrix33<> principal_inertia_rot;
//		ChVector<> principal_I;
//		ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);
//
//		auto body = chrono_types::make_shared<ChBodyAuxRef>();
//		sys.Add(body);
//		body->SetBodyFixed(false);
//		body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos[i]), QUNIT));
//		body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));
//		body->SetMass(mass * rock_density);
//		body->SetInertiaXX(rock_density * principal_I);
//
//		body->GetCollisionModel()->ClearModel();
//		body->GetCollisionModel()->AddTriangleMesh(rock_mat, mesh, false, false, VNULL, ChMatrix33<>(1),
//												   0.005);
//		body->GetCollisionModel()->BuildModel();
//		body->SetCollide(true);
//
//		auto mesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
//		mesh_shape->SetMesh(mesh);
//		mesh_shape->SetBackfaceCull(true);
//		body->AddVisualShape(mesh_shape);
//
//		rock.push_back(body);
//	}

	// Create the SCM deformable terrain
//	terrain = std::make_unique<vehicle::SCMTerrain>(&sys);
	
	CreateTerrain(sys);

	// Displace/rotate the terrain reference plane.
	// Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
	// a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
//	terrain->SetPlane(ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(-CH_C_PI_2)));
	
	// Use a regular grid
	double length = 70;
	double width = 20;
	// SCM grid spacing
	double mesh_resolution = 1;

	// Enable/disable bulldozing effects
	bool enable_bulldozing = false;

	// Enable/disable moving patch feature
	bool enable_moving_patch = false;

//	terrain->Initialize(length, width, mesh_resolution);

	// Set the soil terramechanical parameters
//	terrain->SetSoilParameters(0.82e6,   // Bekker Kphi
//							  0.14e4,   // Bekker Kc
//							  1.0,      // Bekker n exponent
//							  0.017e4,  // Mohr cohesive limit (Pa)
//							  35,       // Mohr friction limit (degrees)
//							  1.78e-2,  // Janosi shear coefficient (m)
//							  2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
//							  3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
//							  );

	// Set up bulldozing factors
//	terrain->EnableBulldozing(enable_bulldozing);
//	terrain->SetBulldozingParameters(55,  // angle of friction for erosion of displaced material at the border of the rut
//									1,   // displaced material vs downward pressed material.
//									5,   // number of erosion refinements per timestep
//									6);  // number of concentric vertex selections subject to erosion
//
//	// Enable moving patches (for SCM efficiency)
//	if (enable_moving_patch) {
//		// add moving patch for each rover wheel
//		for (const auto& wheel : rover->GetWheels())
//			terrain->AddMovingPatch(wheel->GetBody(), VNULL, ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
//
//		// add moving patch for each obstacles
//		for (int i = 0; i < 6; i++)
//			terrain->AddMovingPatch(rock[i], VNULL, ChVector<>(2.0, 2.0, 2.0));
//	}

	driver = chrono_types::make_shared<CuriosityMagnosDriver>(*rover, msr::airlib::Vector3r(-5, -0.5f, 0));
	
//	driver = chrono_types::make_shared<CuriositySpeedDriver>(0.5, 12.0);
	
	rover->SetDriver(driver);
	rover->Initialize(ChFrame<>(ChVector<>(-5, -0.5, 0), QUNIT));

	std::cout << "Curiosity total mass: " << rover->GetRoverMass() << std::endl;
	std::cout << "  chassis:            " << rover->GetChassis()->GetBody()->GetMass() << std::endl;
	std::cout << "  wheel:              " << rover->GetWheel(CuriosityWheelID::C_LF)->GetBody()->GetMass() << std::endl;
	std::cout << std::endl;

	
	auto vis_axmol = chrono_types::make_shared<chrono::axmol::ChVisualSystemAxmol>();
	vis_axmol->AttachSystem(&sys);

	vis_axmol->Initialize(this);
	
	vis = vis_axmol;
	
	return true;
}

void RoverGameState::setup(ax::Camera* defaultCamera){
	
	// Create a directional light
	auto directionalLight = DirectionLight::create(Vec3(0, -1, -0.25f), Color3B::WHITE);
	this->addChild(directionalLight);
	auto directionalLight2 = DirectionLight::create(Vec3(0, 1, 0.25f), Color3B::WHITE);
	this->addChild(directionalLight2);

	// Create a point light
	auto pointLight = PointLight::create(Vec3(0, 5, 5), Color3B::WHITE, 1000.0f);
	this->addChild(pointLight);
	
	// Create a spot light
	auto spotLight = SpotLight::create(Vec3(0, 5, 0), Vec3(0, 1, 0), Color3B::WHITE, 30, 45, 1000.0f);
	this->addChild(spotLight);

	
//	auto skybox = ax::Skybox::create("white.jpg", "white.jpg", "white.jpg", "white.jpg", "white.jpg", "white.jpg");
	
//	this->addChild(skybox);
	
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	
	_defaultCamera->setPosition3D(Vec3(5, 5, 5));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0), Vec3(0, 0, 1));
	
	
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
	if(driver->getEngine()->isCalibrating()){
		return;
	}

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

void RoverGameState::onKeyReleased(EventKeyboard::KeyCode code, Event*)
{

	if(driver->getEngine()->isCalibrating()){
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
	// Update Curiosity controls
	
	driver->getEngine()->update(Settings::fixed_delta);

	if(accelerate){
		stallTorque += 1200 * Settings::fixed_delta;
		driver->accelerate(0.5f);
	} else {
		driver->accelerate(0.0f);
	}
	
	rover->Update();
	
	sys.DoStepDynamics(Settings::fixed_delta);

	if(steer){
		driver->SetSteering(steerAngle);
	} else {
		driver->SetSteering(0);
	}

 
	// Get the car's position
	auto roverPosition = rover->GetChassisPos();
	Vec3 carPosition = Vec3(roverPosition.x(), roverPosition.y(), roverPosition.z());
	
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

void RoverGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	

	float laserOutput = 0;
	float laserInput = 0;
	for(auto laserNode : driver->getEngine()->getLasers()){
		laserOutput += laserNode->getGuiMeasure();
		laserInput += laserNode->getVoltageInput();
	}
	
	float coilInput = 0;
	for(auto magnos : driver->getEngine()->getGimbals()){
		coilInput += currentToVoltage(magnos->getCoilSystem().current, Settings::circuit_resistance);
	}
	
	ImGui::Begin("Engine");
	
	auto status = driver->getEngine()->getMagnosFeedback().status;
	
	ImGui::Text("Status=%s", status.c_str());

	ImGui::Text("Consumption (VDC)=%.2f",  driver->getEngine()->isCalibrating() ? 0 : inputAverageFilter.filter( driver->getEngine()->getEngineConsumption()));

	ImGui::End();
	
	ImGui::SetNextWindowPos(ImVec2(960, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Rover");
	
	
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
		battery = driver->getEngine()->getBatteryVoltage();
		float linearAvg = 0.0f;
		linearAvg += rover->GetChassisVel().x();
		linearAvg += rover->GetChassisVel().y();
		linearAvg += rover->GetChassisVel().z();
		
		linearAvg /= 3.0f;
		
//		acceleration = car->getAcceleration();
		speed = linearAvg;
	}
	
	ImGui::Text("Battery Voltage=%.2f", battery);
//	ImGui::Text("Accel m/s^2=%.2f", acceleration);
	ImGui::Text("Speed km/h=%.2f", mpsToKmph(speed));
	ImGui::End();
	

}
