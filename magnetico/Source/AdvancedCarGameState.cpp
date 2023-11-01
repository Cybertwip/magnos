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

#include "TusimpleEngine.h"

#include <memory>



class CustomViewportCamera : public ax::Camera {
public:
	
	CustomViewportCamera() : ax::Camera() {
		_viewport = Camera::getDefaultViewport();
		updateBuffers();
	}
	
	~CustomViewportCamera(){
	}
	
	static CustomViewportCamera* createPerspective(float fieldOfView, float aspectRatio, float nearPlane, float farPlane)
	{
		auto ret = new CustomViewportCamera();
		ret->initPerspective(fieldOfView, aspectRatio, nearPlane, farPlane);
		ret->autorelease();
		return ret;
	}
	
	static CustomViewportCamera* createOrthographic(float zoomX, float zoomY, float nearPlane, float farPlane)
	{
		auto ret = new CustomViewportCamera();
		ret->initOrthographic(zoomX, zoomY, nearPlane, farPlane);
		ret->autorelease();
		return ret;
	}
	
	void applyViewport() override {
		_director->getRenderer()->setViewPort(_viewport.x,
											  _viewport.y,
											  _viewport.width,
											  _viewport.height);
	}
	
	void clearBackground() override {

		if (_clearBrush)
		{
			_clearBrush->drawBackground(this);
		}
		

		_director->getRenderer()->clear(ax::ClearFlag::DEPTH_AND_STENCIL, ax::Color4F::WHITE, 1, 0, getGlobalZOrder());
		
	}
	
	void setCustomViewport(ax::Viewport viewport){
		_viewport = viewport;
		clippedBuffer.width = _viewport.width;
		clippedBuffer.height = _viewport.height;
		clippedBuffer.channels = 4;
		
		clippedBuffer.data.resize(clippedBuffer.width * clippedBuffer.height * clippedBuffer.channels);
	}
	
	void setDepthTest(bool depthTest){
		_depthTest = depthTest;
	}

	void setDepthWrite(bool depthWrite){
		_depthWrite = depthWrite;
	}
	
	void getSnapshot(std::function<void(Image&)> imageCallback){
		ax::utils::captureScreen([imageCallback, this](ax::RefPtr<ax::Image> image){
			memcpy(buffer.data(), image->getData(), image->getDataLen());
			
			// Define the dimensions of the original image
			int originalWidth = image->getWidth();
			int originalHeight = image->getHeight();
			
			// Define the clipping rectangle parameters
			int clipX = _viewport.x;      // X-coordinate of the top-left corner of the clipping region
			int clipY = _viewport.y;      // Y-coordinate of the top-left corner of the clipping region
			int clipWidth = _viewport.width;  // Width of the clipped region
			int clipHeight = _viewport.height; // Height of the clipped region
			
			for (int y = 0; y < clipHeight; y++) {
				for (int x = 0; x < clipWidth; x++) {
					int srcX = clipX + x;
					int srcY = originalHeight - (clipY + y) - 1; // Flip Y-coordinate
					if (srcX >= 0 && srcX < originalWidth && srcY >= 0 && srcY < originalHeight) {
						// Calculate the source and destination buffer offsets
						int srcOffset = (srcY * originalWidth + srcX) * 4;
						int destOffset = ((clipHeight - 1 - y) * clipWidth + x) * 4; // Vertical flip
						
						// Copy the pixel data (assuming RGBA format, 4 bytes per pixel)
						for (int i = 0; i < 4; i++) {
							clippedBuffer.data[destOffset + i] = buffer[srcOffset + i];
						}
					}
				}
			}
						
			imageCallback(clippedBuffer);
		});
			
	}

private:
	void updateBuffers(){
		buffer = std::vector<uint8_t>(_viewport.width * _viewport.height * 4);
		
		clippedBuffer.width = _viewport.width;
		clippedBuffer.height = _viewport.height;
		clippedBuffer.channels = 4;
		
		clippedBuffer.data.resize(clippedBuffer.width * clippedBuffer.height * clippedBuffer.channels);
	}
	ax::Viewport _viewport;

	bool _depthTest = true;
	bool _depthWrite = true;
	
	std::vector<uint8_t> buffer;
	Image clippedBuffer;

};

namespace{


double xend = 400.0;  // end logging here, this also the end of our world

void CreateGround(ChSystem& sys) {
	// Create the ground and obstacles
	auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
	auto ground = chrono_types::make_shared<ChBodyEasyBox>(xend, xend, 1, 1000, true, true, ground_mat);
	ground->SetPos(ChVector<>(0, 0, -1));
	//ground->SetBodyFixed(true);
	ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
	sys.Add(ground);
}
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

class StellaMagnosDriver : public ChDriver {
public:
	StellaMagnosDriver(ChWheeledVehicle& vehicle, msr::airlib::Vector3r position);
	
	~StellaMagnosDriver() {}
	
	void accelerate(float voltage);
	void steer(float angle);
	void brake(float amount);
	
	std::shared_ptr<EVEngine> getEngine() const {
		return engine_;
	}
	
	void Update(double delta);
private:
	
	double m_ramp;
	double m_speed;
	
	std::shared_ptr<EVEngine> engine_;
//	Curiosity& rover;
	
	std::vector<WheelMotor> wheelMotors_;
};

StellaMagnosDriver::StellaMagnosDriver(ChWheeledVehicle& vehicle, msr::airlib::Vector3r position) : ChDriver(vehicle) {
	engine_ = std::make_shared<EVEngine>(120);
	engine_->setPosition3D(position);
	engine_->init(ax::FileUtils::getInstance()->getWritablePath());
	
	for(int i = 0; i<4; ++i){
		wheelMotors_.push_back(WheelMotor(80, 0.5f));
	}
}

void StellaMagnosDriver::accelerate(float throttle){
	for(auto& motor : wheelMotors_){
		motor.SetVoltage(motor.GetMaxVoltage() * throttle);
	}
}

void StellaMagnosDriver::steer(float angle){
//	m_steering = angle / 360.0f;
	SetSteering(angle);
}

void StellaMagnosDriver::brake(float amount){
	float braking = amount / 1.0f;
	
	SetBraking(braking);
}


void StellaMagnosDriver::Update(double time) {
	auto roverPosition = m_vehicle.GetPos();
	msr::airlib::Vector3r position = msr::airlib::Vector3r(roverPosition.x(), roverPosition.y(), roverPosition.z());
	
	engine_->setPosition3D(position);

	float consumption = 0.0f;
	std::vector<float> angularSpeeds;
	
	for (auto& motor : wheelMotors_) {
		consumption += motor.GetCurrentConsumption();
	}
	
	engine_->setEngineConsumption(consumption);
	
	float throttle = consumption / EVEngine::max_voltage;
	
	SetThrottle(throttle);
	
	// Use angular speeds to set drive_speeds (assuming 6-wheel configuration).
//	for(int i = 0; i<angularSpeeds.size(); ++i){
//		curiosity->GetDriveshaft(static_cast<CuriosityWheelID>(i))->SetAppliedTorque(-angularSpeeds[i]);
//
//	}
	
	
}
#include "chrono_models/vehicle/duro/Duro_Vehicle.h"

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
ChVector<> initLoc(32, 0, 0.725);

// Logging of seat acceleration data on flat road surface is useless and would lead to distorted results

class InferenceImageBuffer : public ax::Image {
public:
	InferenceImageBuffer(){
		this->_width  = 1280;
		this->_height = 720;
		this->_dataLen = this->_width * this->_height * 4;
		this->_data = (uint8_t*)malloc(this->_dataLen);
		this->_pixelFormat = ax::PixelFormat::RGBA8;
		memset(this->_data, 0, this->_dataLen);
	}
	
	virtual ~InferenceImageBuffer() override {
		free(this->_data);
		
		this->_data = nullptr;
	}
};

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

AdvancedCarGameState::AdvancedCarGameState() {
	// Initialize private members and resources specific to the rover simulation
	
	_mainLayer = ax::Layer::create();
	_mainLayer->retain();
	_secondaryLayer = ax::Layer::create();
	_secondaryLayer->retain();
	_2dLayer = ax::Layer::create();
	_2dLayer->retain();

	_snapshotBuffer = new InferenceImageBuffer();
	

}

AdvancedCarGameState::~AdvancedCarGameState(){
	_mainLayer->release();
	_secondaryLayer->release();
	_2dLayer->release();
	_visionRenderer->release();
	
	delete(_snapshotBuffer);

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

	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

	std::string vehicleDataPath =
	std::string{ CHRONO_DATA_DIR } + "vehicle/";
	
	vehicle::SetDataPath(vehicleDataPath.c_str());
	int iTire = 3;
	
	double target_speed = 420.0;
	
	// JSON files for terrain
	std::string rigidterrain_file("terrain/RigidMeshFrisco.json");
	
	GetLog() << "\n"
	<< "Speed       = " << target_speed << " m/s\n"
	<< "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka89, 4=Pacejka02, 5=Rigid) = " << iTire << "\n";

	// --------------------------
	// Create the various modules
	// --------------------------
	ChContactMethod contact_method = ChContactMethod::NSC;
	
//	vehicle = std::make_unique<duro::Duro_Vehicle>(false,
//		BrakeType::SHAFTS,
//	 	SteeringTypeWV::PITMAN_ARM_SHAFTS,
//		contact_method,
//		CollisionType::PRIMITIVES);
	
	vehicle = std::make_unique<
	WheeledVehicle>(vehicle::GetDataFile(vehicle_file), contact_method);
	
	vehicle->Initialize(ChCoordsys<>(initLoc, QUNIT));
	vehicle->GetChassis()->SetFixed(false);
	vehicle->SetChassisVisualizationType(VisualizationType::MESH);
	vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	vehicle->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
	
	auto terrainImpl = std::make_unique<RigidTerrain>(vehicle->GetSystem(), vehicle::GetDataFile(rigidterrain_file));

	
//	terrainImpl->Initialize();
		
	terrain = std::move(terrainImpl);
	
	// Create and initialize the powertrain system
	auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
	
	auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
	auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
	vehicle->InitializePowertrain(powertrain);
	
	
	vehicle->SetChassisCollide(true);
	vehicle->SetChassisVehicleCollide(true);

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
	auto vehiclePosition = vehicle->GetPos();
	msr::airlib::Vector3r position = msr::airlib::Vector3r(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());
	driver = std::make_shared<StellaMagnosDriver>(*vehicle, position);
	driver->Initialize();
	
	auto vis_axmol = chrono_types::make_shared<chrono::axmol::ChVisualSystemAxmol>();
	vis_axmol->AttachSystem(vehicle->GetSystem());
	vis_axmol->Initialize(_mainLayer);
	
	vis = vis_axmol;
	
	auto frisco = ax::MeshRenderer::create("frisco/frisco.c3b");

	auto material = ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false);

	for(auto mesh : frisco->getMeshes()){
		auto friscoRenderer = ax::MeshRenderer::create();
		
		if(mesh->getMaterial() == nullptr){
			mesh->setMaterial(material);
			mesh->setTexture("gray.jpg");
		} else {
			
		}
		
		friscoRenderer->addMesh(mesh);
		
		friscoRenderer->setMaterial(mesh->getMaterial());
		
		_mainLayer->addChild(friscoRenderer);

	}

	auto rotation = vehicle->GetChassis()->GetRot();
	
	ax::Quaternion quaternion = ax::Quaternion(rotation.e1(),
					   rotation.e2(),
					   rotation.e3(),
					   rotation.e0());
	
	_inferenceEngine = std::make_unique<TusimpleEngine>();

	_visionRenderer = ax::Sprite::create("dummy.png");
	
	_visionRenderer->retain();
	
	_visionRenderer->setContentSize(ax::Vec2(_visionRenderer->getTextureRect().size.width / 4 * ax::Director::getInstance()->getContentScaleFactor(), _visionRenderer->getTextureRect().size.height / 4 * ax::Director::getInstance()->getContentScaleFactor()));
	
	_visionRenderer->setAnchorPoint(ax::Vec2(0, 0));
	_visionRenderer->setPosition(ax::Vec2(0, 0));
		
	_inputInferenceBuffer = std::make_unique<Image>();
	
	_inputInferenceBuffer->width = 1280;
	_inputInferenceBuffer->height = 720;
	_inputInferenceBuffer->channels = 4;
		
	_inputInferenceBuffer->data.resize(_inputInferenceBuffer->width * _inputInferenceBuffer->height * _inputInferenceBuffer->channels);
		
	_backgroundTask = std::thread([this](){
		while(this->running){
			
			
			_inferenceMutex.lock();
			Image inferenceDataCopy = *_inputInferenceBuffer;
			_inferenceMutex.unlock();

			auto [processed, points, lanes] = _inferenceEngine->detectLanes(inferenceDataCopy);
						
			_snapshotMutex.lock();
			memcpy(_snapshotBuffer->getData(), processed.data.data(), processed.data.size());
			_snapshotMutex.unlock();
			
			// Signal that new data is available
//			newDataAvailable = true;
//			dataCondition.notify_all();
			
			
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 fps

		}
	});

	return true;
}

void AdvancedCarGameState::onEnter(){
	GameState::onEnter();
}

void AdvancedCarGameState::onExit(){
	GameState::onExit();

	this->running = false;
	
	_backgroundTask.join();
	
}

void AdvancedCarGameState::setup(ax::Camera* defaultCamera){
	
	// Create a directional light
	auto directionalLight = ax::DirectionLight::create(ax::Vec3(0, 0, -1), ax::Color3B::WHITE);
	_mainLayer->addChild(directionalLight);
	auto directionalLight2 = ax::DirectionLight::create(ax::Vec3(0, 0, 1), ax::Color3B::WHITE);
	
	_mainLayer->addChild(directionalLight2);

	// Create a point light
	auto pointLight = ax::PointLight::create(ax::Vec3(0, 5, 5), ax::Color3B::WHITE, 1000.0f);
	_mainLayer->addChild(pointLight);
	
	
	auto winSize = ax::Camera::getDefaultViewport();
	
	ax::Viewport viewport;
	viewport.set((float)winSize.width * 0.5f, 0, (float)winSize.width * 0.5f, (float)winSize.height * 0.5f);

	_secondaryCamera = CustomViewportCamera::createPerspective(90, (float)viewport.width / viewport.height, 1, 1000);

	_primaryCamera = CustomViewportCamera::createPerspective(90, (float)ax::Director::getInstance()->getWinSize().width / ax::Director::getInstance()->getWinSize().height, 1, 1000);

	defaultCamera->setVisible(false);
	defaultCamera->setDepth(-1);
	
	_primaryCamera->setCustomViewport(ax::Camera::getDefaultViewport());
	
	_secondaryCamera->setCustomViewport(viewport);
//	_secondaryCamera->setCustomViewport(Camera::getDefaultViewport());

	_primaryCamera->setNearPlane(0.01f);
	_primaryCamera->setFarPlane(10000);
	_primaryCamera->setFOV(90);
	_primaryCamera->setZoom(1);
	
	_primaryCamera->setPosition3D(ax::Vec3(5, 5, 5));
	_primaryCamera->setRotation3D(ax::Vec3(0, 0, 0));
	_primaryCamera->lookAt(ax::Vec3(0, 0, 0), ax::Vec3(0, 0, 1));
	
	_secondaryCamera->setNearPlane(0.01f);
	_secondaryCamera->setFarPlane(10000);
	_secondaryCamera->setFOV(90);
	_secondaryCamera->setZoom(1);

	_secondaryCamera->setPosition3D(ax::Vec3(5, 5, 5));
	_secondaryCamera->setRotation3D(ax::Vec3(0, 0, 0));
	_secondaryCamera->lookAt(ax::Vec3(0, 0, 0), ax::Vec3(0, 0, 1));

	_primaryCamera->setDepth(0);
	_primaryCamera->setCameraFlag(ax::CameraFlag::DEFAULT);

	_secondaryCamera->setDepth(4);
	_secondaryCamera->setCameraFlag(ax::CameraFlag::USER1);
	
	auto blue = ax::Color4F::BLUE;
	
	_secondaryCamera->setBackgroundBrush(ax::CameraBackgroundBrush::createColorBrush(blue, 1));
	
	this->addChild(_mainLayer);
	
	_mainLayer->setCameraMask((unsigned short)ax::CameraFlag::DEFAULT | (unsigned short)ax::CameraFlag::USER1);
	_secondaryLayer->setCameraMask((unsigned short)ax::CameraFlag::DEFAULT | (unsigned short)ax::CameraFlag::USER1);
	_2dLayer->setCameraMask((unsigned short)ax::CameraFlag::DEFAULT | (unsigned short)ax::CameraFlag::USER1);

	getScene()->addChild(_primaryCamera);
	getScene()->addChild(_secondaryCamera);
}

void AdvancedCarGameState::onMouseMove(ax::Event* event)
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


void AdvancedCarGameState::onKeyPressed(ax::EventKeyboard::KeyCode code, ax::Event*)
{
	if(driver->getEngine()->isCalibrating()){
		return;
	}

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

void AdvancedCarGameState::onKeyReleased(ax::EventKeyboard::KeyCode code, ax::Event*)
{

	if(driver->getEngine()->isCalibrating()){
		return;
	}

	if(code == ax::EventKeyboard::KeyCode::KEY_SPACE){
		accelerate = false;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_RIGHT_ARROW || code == ax::EventKeyboard::KeyCode::KEY_LEFT_ARROW){
		steer = false;
		steerAngle = 0;
	}
	
	if(code == ax::EventKeyboard::KeyCode::KEY_DOWN_ARROW){
		brake = false;
	}
}

void AdvancedCarGameState::update(float delta) {
	driver->getEngine()->update(delta);

	if(accelerate){
		driver->accelerate(0.5f);
	} else {
		driver->accelerate(0.0f);
	}

	if(steer){
		driver->steer(steerAngle);
	} else {
		driver->steer(0);
	}
	
	
	if(brake){
		float brakePedalInput = 1.0f; // Adjust as needed
		driver->brake(brakePedalInput);
	} else {
		float brakePedalInput = 0; // Adjust as needed
		driver->brake(brakePedalInput);
	}

	driver->Update(delta);

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
	
	// Get the car's position
	auto vehiclePosition = vehicle->GetPos();
	auto chassisPosition = vehicle->GetChassis()->GetPos();
	auto rotation = vehicle->GetChassis()->GetRot();
	
//	carMesh_->setScale(1.215f);
//	carMesh_->setScaleZ(1.200f);
//	carMesh_->setPosition3D(Vec3(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z()));
	
	// Original quaternion from rotation.e1(), e2(), e3(), e0()
	ax::Quaternion originalQuaternion = ax::Quaternion(rotation.e1(), rotation.e2(), rotation.e3(), rotation.e0());
	
	// Z-Up rotation quaternion (90 degrees around the X-axis)
//	float zUpAngle = M_PI_2; // 90 degrees in radians
//	ax::Quaternion zUpRotation = ax::Quaternion(sin(zUpAngle / 2), 0, 0, cos(zUpAngle / 2));
//
	// Yaw rotation quaternion (45 degrees around the Z-axis)
	float yawAngle = M_PI; // 45 degrees in radians
	ax::Quaternion yawRotation = ax::Quaternion(0, 0, sin(yawAngle / 2), cos(yawAngle / 2));
	
	// Multiply the original quaternion by the Z-Up and Yaw rotations
	ax::Quaternion finalQuaternion = yawRotation * originalQuaternion;
	
	// Set the final quaternion for the carMesh_
//	carMesh_->setRotationQuat(finalQuaternion);

	ax::Vec3 carPosition = ax::Vec3(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());

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

	ax::Vec3 cameraOffset(horizontalOffset, depthOffset, cameraHeight + verticalOffset);

	// Calculate the new camera position
	ax::Vec3 newPosition = carPosition + cameraOffset;
	
	// Set the camera's new position and look-at point
	_primaryCamera->setPosition3D(newPosition);
	_primaryCamera->lookAt(carPosition, ax::Vec3(0, 0, 1));
	
	ax::Vec3 target = carPosition + originalQuaternion * ax::Vec3(4, 0, 4);
	ax::Vec3 cameraPosition = carPosition + originalQuaternion * ax::Vec3(0, 0, 4);
	_secondaryCamera->setRotationQuat(originalQuaternion);
	_secondaryCamera->setPosition3D(cameraPosition);
	_secondaryCamera->lookAt(target, ax::Vec3(0, 0, 1));

	cursorDeltaX = 0;
	cursorDeltaY = 0;
	

	_secondaryCamera->getSnapshot([this](Image& image){
		
		_inferenceMutex.lock();
		memcpy(_inputInferenceBuffer->data.data(), image.data.data(), image.data.size());
		_inferenceMutex.unlock();
		
		_visionRenderer->getTexture()->updateWithImage(_snapshotBuffer, ax::PixelFormat::RGBA8);
		
		_visionRenderer->visit(ax::Director::getInstance()->getRenderer(), ax::Mat4::IDENTITY, 0);

	});


}

void AdvancedCarGameState::renderUI() {
	ImGui::Begin("Engine");
	
	float laserOutput = 0;
	float laserInput = 0;
	
	for(auto laserNode : driver->getEngine()->getLasers()){
		laserOutput += laserNode->getGuiMeasure();
		laserInput += laserNode->getVoltageInput();
	}
	
	float coilInput = 0;
	for(auto magnos : driver->getEngine()->getGimbals()){
		coilInput += currentToVoltage(magnos->getCoilSystem().current * Settings::fixed_delta, Settings::circuit_resistance);
	}

	auto status = driver->getEngine()->getMagnosFeedback().status;
	
	ImGui::Text("Status=%s", status.c_str());
	
	ImGui::Text("Input Voltage=%.2f",  driver->getEngine()->isCalibrating() ? 0 : inputAverageFilter.filter(laserInput + coilInput));

	ImGui::Text("Base + Gain Voltage=%.4f",
				outputAverageFilter.filter(driver->getEngine()->getMagnosFeedback().baseEMF + driver->getEngine()->getMagnosFeedback().EMF + laserOutput));

	ImGui::Text("Consumption (VDC)=%.2f",  driver->getEngine()->isCalibrating() ? 0 : engineAverageFilter.filter( driver->getEngine()->getEngineConsumption()));
	
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
	
	if(counter >= Settings::fixed_delta * 60.0f){
		counter = 0;
		battery = 0;
		laser = 0;
		battery = driver->getEngine()->getBatteryVoltage();
		float linearAvg = 0.0f;
		linearAvg += vehicle->GetSpeed();
		
		//		acceleration = car->getAcceleration();
		speed = linearAvg;
		
		
		for(auto laserNode : driver->getEngine()->getLasers()){
			laser += laserNode->getGuiMeasure();
		}

	}
	
	ImGui::Text("Battery Voltage=%.2f", battery);
	ImGui::Text("Speed km/h=%.2f", mpsToKmph(speed));
	ImGui::Text("Laser v/s=%.2f", laser);
	ImGui::End();
}
