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

#include "core/physics3d/Physics3D.h"

#include "ui/axmol-ui.h"

#include "ImGui/ImGuiPresenter.h"

#include <PID_AutoTune_v0.h>

#include <fstream>

#include <mlpack.hpp>
//#include <mlpack/methods/linear_regression/linear_regression.hpp>


USING_NS_AX;
USING_NS_AX_EXT;

// Define direction flags
enum class MagnetDirection {
	NORTH,
	NORTHEAST,
	EAST,
	SOUTHEAST,
	SOUTH,
	SOUTHWEST,
	WEST,
	NORTHWEST,
	FRONT,
	BACK
};

enum class MagnetPolarity {
	NORTH,
	SOUTH
};

namespace {
float quaternionDot(const Quaternion& q1, const Quaternion& q2) {
	// Manually compute the dot product
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

const float desired_voltage = 12.009;
const float calibration_voltage = 3;
float error_trial = 0.005f;
const float global_timestep = 60.0f;
const int calibration_steps = 10;
const int calibration_time = 1000;
float global_delta = 1.0f / global_timestep;

const bool adaptive_calibration = true;

const float desired_voltage_per_second = desired_voltage;
}

class ElectromagneticEntity {
protected:
	Mesh* createCube(float dimension) {
		
		// Vertices for a cube
		std::vector<float> positions = {
			-dimension, -dimension, dimension,
			dimension, -dimension, dimension,
			dimension,  dimension, dimension,
			-dimension,  dimension, dimension,
			-dimension, -dimension, -dimension,
			dimension, -dimension, -dimension,
			dimension,  dimension, -dimension,
			-dimension,  dimension, -dimension
		};
		
		// Normals for the cube (not normalized, for the sake of simplicity)
		std::vector<float> normals = {
			0, 0, 1,
			0, 0, 1,
			0, 0, 1,
			0, 0, 1,
			0, 0, -1,
			0, 0, -1,
			0, 0, -1,
			0, 0, -1
		};
		
		// Texture coordinates for the cube
		std::vector<float> texs = {
			0, 0,
			1, 0,
			1, 1,
			0, 1,
			0, 0,
			1, 0,
			1, 1,
			0, 1
		};
		
		// Indices for the cube
		std::vector<unsigned short> indices = {
			0, 1, 2, 0, 2, 3,   // Front face
			5, 4, 7, 5, 7, 6,   // Back face
			4, 0, 3, 4, 3, 7,   // Left face
			1, 5, 6, 1, 6, 2,   // Right face
			4, 5, 1, 4, 1, 0,   // Bottom face
			3, 2, 6, 3, 6, 7    // Top face
		};
		
		IndexArray indexArray;
		
		for(auto index : indices){
			indexArray.emplace_back(index);
		}
		
		// Create mesh and attach to Sprite3D
		return Mesh::create(positions, normals, texs, indexArray);
	}
	
public:
	virtual Vec3 combineFieldsOrForces() = 0;
	virtual void attachToDisk(Node* disk, float radius, MagnetDirection direction, MagnetPolarity polarity) = 0;
	
	virtual ~ElectromagneticEntity() = default;

};

class MagnetEntity : public ElectromagneticEntity {
public:
	struct AttachedEntity {
		Vec3 position;
		MagnetPolarity polarity;
	};

protected:
	std::vector<AttachedEntity> _attachedEntities;
	

public:
	virtual ~MagnetEntity() = default;

	
	std::vector<AttachedEntity>& getAttachedEntities() { return _attachedEntities; }

};


class CoilEntity : public ElectromagneticEntity {
public:
	struct AttachedEntity {
		Vec3 position;
		MagnetPolarity polarity;
		float area = 0;  // Area of the coil.
		float turns = 0;  // Number of turns in the coil.
		float flux = 0;
		float previousFlux = 0;
	};
	
protected:
	std::vector<AttachedEntity> _attachedEntities;
	
	
public:
	virtual ~CoilEntity() = default;
	
	
	std::vector<AttachedEntity>& getAttachedEntities() { return _attachedEntities; }
	
};

class Magnet : public ax::Node {
public:
	Magnet(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
		this->scheduleUpdate();
	}
	
	static Magnet* create(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index) {
		Magnet* magnet = new Magnet(entities, index);
		if (magnet && magnet->init()) {
			magnet->autorelease();
			return magnet;
		}
		delete magnet;
		return nullptr;
	}
	void updatePositions(){
		attachedEntities[entityIndex - 1].position = getWorldPosition3D();
	}
	
private:
	std::vector<MagnetEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};


class Coil : public ax::Node {
public:
	Coil(std::vector<CoilEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
		this->scheduleUpdate();
	}
	
	static Coil* create(std::vector<CoilEntity::AttachedEntity>& entities, size_t index) {
		Coil* coil = new Coil(entities, index);
		if (coil && coil->init()) {
			coil->autorelease();
			return coil;
		}
		delete coil;
		return nullptr;
	}
	void updatePositions(){
		attachedEntities[entityIndex - 1].position = getWorldPosition3D();
	}
	
private:
	std::vector<CoilEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};

class MagnetSystem : public MagnetEntity {
private:
	static constexpr float mu_0 = 4 * M_PI * 1e-7;  // Vacuum permeability
	//float _magneticFieldStrength = 0.0015f;  // T, approximate value for N52 neodymium bar magnet
	float _magneticFieldStrength = 0.003f;  // T, approximate value for a common ferrite magnet

	std::vector<Magnet*> magnets;

public:

	MagnetSystem() {
	}
	
	~MagnetSystem() override = default;
	
	Vec3 calculateMagneticFieldAtOrigin(Vec3 magnetPosition, MagnetPolarity polarity) {
		float distance = magnetPosition.length();
		
		if (distance < 0.001f)
			distance = 0.001f;
		
		// Calculate the magnetic field strength based on the magnet's distance.
		// The field strength will be inversely proportional to the cube of the distance.
		Vec3 magneticField = _magneticFieldStrength * (1.0f / (distance * distance * distance)) * magnetPosition.getNormalized();
		
		// Considering polarity to determine the direction of the magnetic moment
		if(polarity == MagnetPolarity::SOUTH) {
			magneticField = -magneticField;
		}
		
		return magneticField;
	}
	
	Vec3 calculateForceDueToMagnet(const Vec3& magnetPosition, const Vec3& affectedMagnetPosition, MagnetPolarity polarity) {
		Vec3 field = calculateMagneticFieldAtOrigin(magnetPosition - affectedMagnetPosition, polarity);
		return field;
	}

	Vec3 combineFieldsOrForces() override {
		Vec3 totalForce(0, 0, 0);
		
		for (const auto& [magnetPosition, polarity] : _attachedEntities) {
			// Modify force calculation based on polarity if required.
			// For simplicity, assuming polarity does not change the force calculation for now.
			totalForce += calculateMagneticFieldAtOrigin(magnetPosition, polarity);
		}
		
		return totalForce;
	}

	
	void attachToDisk(Node* disk, float radius, MagnetDirection direction, MagnetPolarity polarity) override {
		Vec3 position(0, 0, 0);
		
		switch (direction) {
			case MagnetDirection::NORTH:
				position = Vec3(0, radius, 0);
				break;
			case MagnetDirection::NORTHEAST:
				position = Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::EAST:
				position = Vec3(radius, 0, 0);
				break;
			case MagnetDirection::SOUTHEAST:
				position = Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::SOUTH:
				position = Vec3(0, -radius, 0);
				break;
			case MagnetDirection::SOUTHWEST:
				position = Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::WEST:
				position = Vec3(-radius, 0, 0);
				break;
			case MagnetDirection::NORTHWEST:
				position = Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;

			case MagnetDirection::FRONT:
				position = Vec3(0, 0, -radius);
				break;
				
			case MagnetDirection::BACK:
				position = Vec3(0, 0, radius);
				break;

		}
		
		AttachedEntity coilEntity = {position, polarity};
		_attachedEntities.push_back(coilEntity);


		Mesh* magnetMesh = createCube(radius / 24);
		auto magnetRenderer = MeshRenderer::create();
		magnetRenderer->addMesh(magnetMesh);
		magnetRenderer->setPosition3D(Vec3(0, 0, 0));
		magnetRenderer->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		magnetRenderer->setTexture("black.jpg");
		
		auto magnet = Magnet::create(_attachedEntities, _attachedEntities.size());
		
		magnet->setPosition3D(position);
		magnet->addChild(magnetRenderer);
		
		disk->addChild(magnet);
		
		magnets.push_back(magnet);
	}
	
	void update(){
		for(auto magnet : magnets){
			magnet->updatePositions();
		}
	}

};

class CoilSystem : public CoilEntity {
public:
	float coilResistance;    // Resistance of the coil in Ohms
	float maxCurrent;
	float emf;
	
	float current = 0;
	float turns;
	
	std::vector<Coil*> coils;
	
public:
	CoilSystem(float voltage, float resistance, float current, float coilTurns)
	: coilResistance(resistance), maxCurrent(current), turns(coilTurns){
		setCurrentFromVoltage(voltage);
		
		// Configurations
		tuner.SetOutputStep(15);
		tuner.SetControlType(1);
		tuner.SetNoiseBand(1.0);
		tuner.SetLookbackSec(0.016);  // Reducing from 10 to 1 second.
				
		// Get current time in seconds since epoch
		auto now = std::chrono::system_clock::now();
		auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
		
		previousTime = epochTime;
		
		nowTime = epochTime;

		std::string home = getenv("HOME");

		hasML = loadDataAndTrainModel(home + "/calibration.bin");
		
		if(!hasML){
			saveDataToBinary(home + "/calibration.bin");
		}

		loadPID(home + "/pid.bin");

		pidCurrent.startAutoTuning(maxCurrent, 0);

	}
	
	~CoilSystem(){
		std::string home = getenv("HOME");

		saveDataToBinary(home + "/calibration.bin");
		savePIDToBinary(home + "/pid.bin");
	}

	// Function to retrain model
	void retrainModel() {
		// Load new data
		// This could be from a file, a stream, etc.
		std::string home = getenv("HOME");
		
		saveDataToBinary(home + "/calibration.bin");

		hasML = loadDataAndTrainModel(home + "/calibration.bin");
	}

	void saveDataToBinary(const std::string& filename) {
		std::ofstream outFile(filename, std::ios::binary);
		
		for (const DataPoint& data : dataCollection) {
			outFile.write(reinterpret_cast<const char*>(&data), sizeof(DataPoint));
		}
		
		outFile.close();
	}
	
	void savePIDToBinary(const std::string& filename) {
		std::ofstream outFile(filename, std::ios::binary);
		
		outFile.write(reinterpret_cast<const char*>(&trainedPID), sizeof(trainedPID));
		outFile.close();
	}

	std::unique_ptr<mlpack::regression::LinearRegression> mlModel;
	
	bool hasML = false;
	
	bool loadDataAndTrainModel(const std::string& filename) {
		// Check if the file exists
		std::ifstream testFile(filename);
		if (!testFile) {
			std::cerr << "File does not exist: " << filename << std::endl;
			return false;  // return unsuccessful load and training
		}
		testFile.close();
		
		std::ifstream inFile(filename, std::ios::binary);
		
		std::vector<DataPoint> loadedData;
		
		while (inFile) {
			DataPoint data;
			inFile.read(reinterpret_cast<char*>(&data), sizeof(DataPoint));
			if(inFile) {
				loadedData.push_back(data);
			}
		}
		
		inFile.close();
		
		dataCollection = loadedData;
		
		// Convert loadedData to arma::mat format for mlpack
		arma::mat dataset(4, loadedData.size());
		
		for (size_t i = 0; i < loadedData.size(); i++) {
			dataset(0, i) = loadedData[i].emfError;
			dataset(1, i) = loadedData[i].desiredCurrent;
			dataset(2, i) = loadedData[i].currentAdjustment;
			dataset(3, i) = loadedData[i].finalCurrent;
		}
		
		// Assuming last row (currentAdjustment) as labels and others as features
		arma::mat labels = dataset.row(3);
		dataset.shed_row(3);
		
		// Train a linear regression model
		mlModel = std::make_unique<mlpack::regression::LinearRegression>(dataset, labels);
		
		return true;  // return successful load and training
	}


	bool loadPID(const std::string& filename) {
		// Check if the file exists
		std::ifstream testFile(filename);
		if (!testFile) {
			std::cerr << "File does not exist: " << filename << std::endl;
			return false;  // return unsuccessful load and training
		}
		testFile.close();
		
		std::ifstream inFile(filename, std::ios::binary);
				
		PID data;
		inFile.read(reinterpret_cast<char*>(&data), sizeof(PID));
		inFile.close();
		
		pidCurrent = PIDController(data.kp, data.ki, data.kd);
		
		return true;  // return successful load and training
	}


	void setCurrentFromVoltage(float voltage) {
		this->current = voltage / (coilResistance * numberOfCoils);
		this->current = std::clamp(this->current, 0.0f, maxCurrent);  // Ensure it doesn't exceed maxCurrent
	}

	
	
	Vec3 computeMagneticField(AttachedEntity& coil, const Vec3& point, MagnetPolarity polarity) const {
		Vec3 direction = (point).getNormalized();
		float distance = Vec3(0, 0, 0).distance(point);
		float magnitude = (this->current * coil.turns) / (2 * M_PI * distance);
		
		// Reverse the direction if the polarity is SOUTH
		if(polarity == MagnetPolarity::SOUTH) {
			magnitude = -magnitude;
		}
		
		return direction * magnitude;
	}
	
	
	Vec3 combineFieldsOrForces() override {
		Vec3 totalField(0, 0, 0);
		
		for (auto& coil : _attachedEntities) {
			Vec3 field = computeMagneticField(coil, coil.position, coil.polarity);
			totalField += field;
		}
		
		return totalField;
	}
	
	void attachToDisk(Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) override {
		
		Vec3 position(0, 0, 0);
		
		switch (direction) {
			case MagnetDirection::NORTH:
				position = Vec3(0, radius, 0);
				break;
			case MagnetDirection::NORTHEAST:
				position = Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::EAST:
				position = Vec3(radius, 0, 0);
				break;
			case MagnetDirection::SOUTHEAST:
				position = Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::SOUTH:
				position = Vec3(0, -radius, 0);
				break;
			case MagnetDirection::SOUTHWEST:
				position = Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::WEST:
				position = Vec3(-radius, 0, 0);
				break;
			case MagnetDirection::NORTHWEST:
				position = Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::FRONT:
				position = Vec3(0, 0, -radius);
				break;
				
			case MagnetDirection::BACK:
				position = Vec3(0, 0, radius);
				break;

		}
		
		AttachedEntity coilEntity = {position, polarity, 0, turns};
		_attachedEntities.emplace_back(coilEntity);
		
		Mesh* magnetMesh = createCube(radius / 16);
		auto magnetRenderer = MeshRenderer::create();
		magnetRenderer->addMesh(magnetMesh);
		magnetRenderer->setPosition3D(Vec3(0, 0, 0));
		magnetRenderer->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		magnetRenderer->setTexture("black.jpg");
		
		auto coil = Coil::create(_attachedEntities, _attachedEntities.size());
		
		coil->setPosition3D(position);
		coil->addChild(magnetRenderer);
		
		node->addChild(coil);
	
		coils.push_back(coil);
	}
	
	bool calibration = true;
	bool adaptive = adaptive_calibration;

	bool calibrating() {
		return calibration;
	}

	bool adapting() {
		return adaptive;
	}

private:
	
	struct DataPoint {
		float emfError;
		float desiredCurrent;
		float currentAdjustment;
		float finalCurrent;
	};
	
	struct PID {
		float kp;
		float ki;
		float kd;
	};


	std::vector<DataPoint> dataCollection;
	PID trainedPID;

	class PIDController {
	public:
		float kp, ki, kd;

	private:
		float integral = 0.0f;
		float prev_error = 0.0f;
		bool autoTuning = false;
		bool relayState = false; // Used for relay feedback, true for high, false for low
		float relayHigh = 1.0;   // Set a high relay value
		float relayLow = 0.0;   // Set a low relay value
		double period = 0.0f;
		double prevSwitchTime = 0.0f;
		double oscillationStartTime = 0.0f;
		int oscillationsCount = 0;
		
		
	public:
		// Constructor
		PIDController(float kp, float ki, float kd)
		: kp(kp), ki(ki), kd(kd) {
		}
		
		void startAutoTuning(float relayHigh, float relayLow) {
			autoTuning = true;
			relayState = !relayState;
			this->relayHigh = relayHigh;
			this->relayLow = relayLow;
		}
		
		bool calibrating() {
			return autoTuning;
		}
		
		bool getRelayState() {
			return relayState;
		}
		
		bool calibrate(float error, float dt, long long currentTime){
			if (autoTuning) {
				if ((relayState && error < 0) || (!relayState && error > 0)) {
					// Capture the oscillation period
					if (oscillationsCount > 0) {
						period += currentTime - prevSwitchTime;
					} else if (oscillationsCount == 0) {
						oscillationStartTime = currentTime;
					}
					oscillationsCount++;
					prevSwitchTime = currentTime;
					
					relayState = !relayState;
					
					if (oscillationsCount >= calibration_steps) {  // consider 5 oscillations for averaging
						autoTuning = false;
						period /= (oscillationsCount - 1);  // Average period
						// Ziegler-Nichols method
						kp = 0.6 * relayHigh / period;
						ki = 2 * kp / period;
						kd = kp * period / 8;
					}
				}
			}
			
			auto _ = compute(error, dt);
			
			return autoTuning;

		}
		
		float compute(float error, float dt) {
			
			// Proportional term
			float p = kp * error;
			
			// Integral term with consideration of dt
			integral += error * dt;
			
			// Derivative term and consider dt
			float d = kd * (error - prev_error) / dt;
			
			// Compute the output
			float output = p + ki * integral + d;
			
			// Save the current error for the next iteration
			prev_error = error;
			
			return output;
		}
	};

	float desiredEMFPerSecond = desired_voltage_per_second;
	const int numberOfCoils = 1;
	const float totalResistance = coilResistance * numberOfCoils;
	float accumulatedEMF = 0.0f;
	float accumulationTime = 0.0f;

	
	double setPoint = 3.0f; // Desired value
	double processVariable = 0; // Current value
	double controllerOutput = 0; // PID output
	
	long long previousTime = 0;
	long long nowTime = 0;

	PID_ATune tuner = PID_ATune(&processVariable, &controllerOutput);
	PIDController pidCurrent = PIDController(1.0f, 0, 0);
	bool isCalibratingUpwards = true; // A flag to determine the calibration direction. Initialize as true if you start by calibrating upwards.

	void adjustCurrentBasedOn(float dt) {
		// Calculate desired current based on EMF error and resistance
		float emfError = desiredEMFPerSecond - accumulatedEMF;
		
		// Get current time in seconds since epoch
		auto now = std::chrono::system_clock::now();
		auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
		
		previousTime = nowTime;
		
		nowTime = epochTime;

		// Compute error for the PID
		float desiredCurrent = emfError / totalResistance;

		float currentAdjustment = 0;
		
		if(pidCurrent.calibrate(desiredCurrent, dt, epochTime)){
									
			currentAdjustment = pidCurrent.getRelayState() ? maxCurrent : 0;
			
		} else {
			
			currentAdjustment = pidCurrent.compute(desiredCurrent, dt);

			currentAdjustment = std::clamp(this->current + currentAdjustment, 0.0f, maxCurrent);

			if(calibration){
				currentAdjustment = 0;
				this->current = 0;
				calibration = false;
				
				std::string home = getenv("HOME");
				savePIDToBinary(home + "/pid.bin");

				if(adaptive_calibration){
					desiredEMFPerSecond = calibration_voltage;
				}
				
				return;
			}
			
			
			DataPoint point;
			point.emfError = emfError;
			point.desiredCurrent = desiredCurrent;
			point.currentAdjustment = currentAdjustment;
			point.finalCurrent = this->current;
			
			dataCollection.push_back(point);
			
			if(hasML){
				// Check if conditions are met to retrain
				if(fabs(emfError) > error_trial) {
					retrainModel();
					
					if(adaptive_calibration){
						// Calibration upwards
						if(isCalibratingUpwards && accumulatedEMF + emfError >= calibration_voltage){
							desiredEMFPerSecond = desired_voltage_per_second;
							isCalibratingUpwards = false;  // Switch calibration direction
							adaptive = true;
						}
						// Calibration downwards
						else if(!isCalibratingUpwards && accumulatedEMF + emfError <= desired_voltage_per_second){
							desiredEMFPerSecond = calibration_voltage;
							isCalibratingUpwards = true;   // Switch calibration direction
							adaptive = true;
						}
					}
				} else {
					adaptive = false;
				}
				// Create a column vector for input features
				arma::mat input(3, 1);
				input(0, 0) = emfError;            // This should be your new emfError value
				input(1, 0) = desiredCurrent;         // Replace with your new desiredCurrent value
				input(2, 0) = currentAdjustment;      // Replace with your new currentAdjustment value
				
				arma::rowvec output;  // Use rowvec instead of mat
				mlModel->Predict(input, output);
				
				// Now `output` contains the predicted finalCurrent value
				float predictedFinalCurrent = output(0, 0);
				
				this->current = predictedFinalCurrent;
				currentAdjustment = this->current;


			}
		}

		
		this->current = currentAdjustment;
		this->current = std::clamp(this->current, 0.0f, maxCurrent);
	}

public:
	float lastAccumulatedEMF = 0.0f;
	
	void update(){
		for(auto coil : coils){
			coil->updatePositions();
		}
	}
	
	void update(float measuredEMF, float delta) {
		accumulatedEMF += fabs(measuredEMF);

		// Get current time in seconds since epoch
		auto now = std::chrono::system_clock::now();
		auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
		
		previousTime = nowTime;
		
		nowTime = epochTime;

		accumulationTime += nowTime - previousTime;
//
//		if(accumulationTime >= global_timestep * calibration_time / global_timestep){
//			adjustCurrentBasedOn(delta);
//			accumulatedEMF = 0.0f;
//			accumulationTime = 0.0f;
//		}
		
		if(accumulatedEMF >= desiredEMFPerSecond || ((accumulationTime >= global_timestep * calibration_time / global_timestep) && calibrating())){
			adjustCurrentBasedOn(delta);
			
			lastAccumulatedEMF = accumulatedEMF;
			accumulatedEMF = 0.0f;
			accumulationTime = 0.0f;
		}
	}
};


class AlternatorSystem : public CoilEntity {
public:
	float emf = 0;
	float current = 0;
	float area = 0;
	float turns = 0;
	
	std::vector<Coil*> coils;

public:
	AlternatorSystem(float coilArea, float coilTurns)
	: area(coilArea), turns(coilTurns){
		
	}
	

	Vec3 computeMagneticField(AttachedEntity& coil, const Vec3& point, MagnetPolarity polarity) const {
		return {};
	}
	
	
	Vec3 combineFieldsOrForces() override {
		return {};
	}
	
	void attachToDisk(Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) override {
		
		Vec3 position(0, 0, 0);
		
		switch (direction) {
			case MagnetDirection::NORTH:
				position = Vec3(0, radius, 0);
				break;
			case MagnetDirection::NORTHEAST:
				position = Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::EAST:
				position = Vec3(radius, 0, 0);
				break;
			case MagnetDirection::SOUTHEAST:
				position = Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::SOUTH:
				position = Vec3(0, -radius, 0);
				break;
			case MagnetDirection::SOUTHWEST:
				position = Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
				break;
			case MagnetDirection::WEST:
				position = Vec3(-radius, 0, 0);
				break;
			case MagnetDirection::NORTHWEST:
				position = Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
				break;

			case MagnetDirection::FRONT:
				position = Vec3(0, 0, -radius);
				break;

			case MagnetDirection::BACK:
				position = Vec3(0, 0, radius);
				break;
		}
		
		_attachedEntities.push_back({position, polarity, area, turns});

		Mesh* magnetMesh = createCube(area);
		auto magnetRenderer = MeshRenderer::create();
		magnetRenderer->addMesh(magnetMesh);
		magnetRenderer->setPosition3D(Vec3(0, 0, 0));
		magnetRenderer->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		magnetRenderer->setTexture("gold.jpg");
		
		auto coil = Coil::create(_attachedEntities, _attachedEntities.size());
		
		coil->setPosition3D(position);
		coil->addChild(magnetRenderer);
		
		node->addChild(coil);
		
		coils.push_back(coil);

	}
	
	void update(){
		for(auto coil : coils){
			coil->updatePositions();
		}
	}

};

class MagneticBall : public MeshRenderer {
protected:
	float radius;
	Physics3DComponent* physicsComponent;
	
	virtual float get_density() const = 0;

	float calculate_mass() const {
		return get_density() * calculate_volume();
	}

	float calculate_permeability() const {
		return get_relative_permeability() * 4.0f * M_PI * 1e-7f;  // Tm/A
	}
		
	Mesh* createSphere(float radius, unsigned int rings, unsigned int sectors) {
		std::vector<float> positions;
		std::vector<float> normals;
		std::vector<float> texs;
		std::vector<unsigned short> indices;
		
		float const R = 1.0f / (float)(rings - 1);
		float const S = 1.0f / (float)(sectors - 1);
		
		for (unsigned int r = 0; r < rings; ++r) {
			for (unsigned int s = 0; s < sectors; ++s) {
				float const y = sin(-M_PI_2 + M_PI * r * R);
				float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
				float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);
				
				texs.push_back(s * S);
				texs.push_back(r * R);
				
				positions.push_back(x * radius);
				positions.push_back(y * radius);
				positions.push_back(z * radius);
				
				normals.push_back(x);
				normals.push_back(y);
				normals.push_back(z);
			}
		}
		
		for (unsigned int r = 0; r < rings - 1; ++r) {
			for (unsigned int s = 0; s < sectors - 1; ++s) {
				// First triangle
				indices.push_back(r * sectors + s);
				indices.push_back((r + 1) * sectors + s);
				indices.push_back((r + 1) * sectors + (s + 1));
				
				// Second triangle
				indices.push_back(r * sectors + s);
				indices.push_back((r + 1) * sectors + (s + 1));
				indices.push_back(r * sectors + (s + 1));
			}
		}
		IndexArray indexArray;
		for (auto index : indices) {
			indexArray.emplace_back(index);
		}
		
		return Mesh::create(positions, normals, texs, indexArray);
	}


public:
	MagneticBall(float r) : MeshRenderer() {
		this->radius = r;
		
		float sphereRadius = this->radius;
		unsigned int sphereRings = 20;
		unsigned int sphereSectors = 20;
		
		Mesh* sphereMesh = createSphere(sphereRadius, sphereRings, sphereSectors);

		sphereMesh->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		
		this->addMesh(sphereMesh);
		this->setTexture("gray.jpg");
		setPosition3D(Vec3(0, 0, 0));
	}
	
	virtual ~MagneticBall() {}
	
	virtual float get_magnetization() const {
		// Default magnetization is 0 for generic MagneticBall
		return 0.0f;
	}

	float calculate_inertia() const {
		return (2.0f/5.0f) * calculate_mass() * std::pow(radius, 2);
	}
	
	Physics3DRigidBody* getPhysics3DRigidBody(){
		auto physicsComponent = dynamic_cast<Physics3DComponent*>(this->getComponent("physics"));
		auto physicsObject = dynamic_cast<Physics3DRigidBody*>(physicsComponent->getPhysics3DObject());
		return physicsObject;

	}

	float getForceDueToMagneticField(const Vec3& B, const Vec3& gradB) const {
		float mu_0 = 4.0f * M_PI * std::pow(10.0f, -7);
		float chi = get_relative_permeability() - 1.0f;
		float V = calculate_volume();

		return mu_0 * V * chi * B.dot(gradB);
	}


	float calculate_volume() const {
		return (4.0f/3.0f) * M_PI * std::pow(radius, 3);
	}
	virtual float get_relative_permeability() const = 0;
};

class IronBall : public MagneticBall {
	static constexpr float density = 7870.0f;  // kg/m^3
	static constexpr float relative_permeability = 5000.0f;  // This is a general value; actual value can vary widely
	static constexpr float M_s = 1.7e6;  // Saturation magnetization for Iron (in A/m)

private:
	IronBall(float radius) : MagneticBall(radius) {
		
	}
	
protected:
	float get_density() const override {
		return density;
	}

	
	float get_relative_permeability() const override {
		return relative_permeability;
	}
	
	float get_magnetization() const override {
		// Return saturation magnetization for Iron
		return M_s;
	}

public:
	static IronBall* create(float radius) {
		IronBall* ball = new IronBall(radius);
		if (ball && ball->init()) {
			ball->autorelease();
			return ball;
		}
		delete ball;
		return nullptr;
	}
};

class CustomNode : public ax::MeshRenderer {
private:
	Quaternion previousRotation;
	float currentAngularSpeed; // New variable to store angular speed
	
public:
	CustomNode() : previousRotation(Quaternion()), currentAngularSpeed(0.0f) {}
	
	// Getter for angular speed
	float getAngularSpeed() const {
		return currentAngularSpeed;
	}
	
	// Setter for angular speed
	void setAngularSpeed(float speed) {
		currentAngularSpeed = speed;
	}
	
	CREATE_FUNC(CustomNode);
};

class MaritimeGimbal3D : public ax::Node {
private:
	
	static constexpr float MU_0 = 4.0f * M_PI * 1e-7f;  // Vacuum permeability
	static constexpr float B0 = 0.01;  // Example value in teslas for a standard lab magnet

	Mesh* createPole(float height, float radius, int segments) {
		std::vector<float> positions;
		std::vector<float> normals;
		std::vector<float> texs;
		std::vector<unsigned short> indices;
		
		float angleStep = 2.0 * M_PI / segments;
		for (int i = 0; i <= segments; i++) {
			float angle = i * angleStep;
			positions.push_back(radius * cosf(angle));
			positions.push_back(-height / 2.0f);
			positions.push_back(radius * sinf(angle));
			
			positions.push_back(radius * cosf(angle));
			positions.push_back(height / 2.0f);
			positions.push_back(radius * sinf(angle));
			
			// Normals (roughly, since we aren't normalizing for simplicity)
			normals.push_back(cosf(angle));
			normals.push_back(0.0f);
			normals.push_back(sinf(angle));
			
			normals.push_back(cosf(angle));
			normals.push_back(0.0f);
			normals.push_back(sinf(angle));
			
			// Texture coords
			texs.push_back((float)i / segments);
			texs.push_back(0.0f);
			
			texs.push_back((float)i / segments);
			texs.push_back(1.0f);
			
			if (i < segments) {
				int offset = i * 2;
				indices.push_back(offset);
				indices.push_back(offset + 1);
				indices.push_back(offset + 2);
				
				indices.push_back(offset + 1);
				indices.push_back(offset + 3);
				indices.push_back(offset + 2);
			}
		}
		
		IndexArray indexArray;
		for (auto index : indices) {
			indexArray.emplace_back(index);
		}
		
		return Mesh::create(positions, normals, texs, indexArray);
	}
public:
	void addRodsToIronBall(IronBall* ball, float rodLength, float rodRadius) {
		auto northRod = ax::MeshRenderer::create();
		auto southRod = ax::MeshRenderer::create();
		auto eastRod = ax::MeshRenderer::create();
		auto westRod = ax::MeshRenderer::create();
		
		auto rodMesh1 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
		auto rodMesh2 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
		auto rodMesh3 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
		auto rodMesh4 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod

		rodMesh1->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		rodMesh2->setMaterial(rodMesh1->getMaterial());
		rodMesh3->setMaterial(rodMesh1->getMaterial());
		rodMesh4->setMaterial(rodMesh1->getMaterial());

		northRod->addMesh(rodMesh1);
		southRod->addMesh(rodMesh2);
		eastRod->addMesh(rodMesh3);
		westRod->addMesh(rodMesh4);
		
		northRod->setTexture("gray.jpg");
		southRod->setTexture("gray.jpg");
		eastRod->setTexture("gray.jpg");
		westRod->setTexture("gray.jpg");
		
		float radius = 0.005f;
		// Positioning the rods
		northRod->setPosition3D(ax::Vec3(0, rodLength / 2 + radius, 0));
		southRod->setPosition3D(ax::Vec3(0, -(rodLength / 2 + radius), 0));
		eastRod->setPosition3D(ax::Vec3(rodLength / 2 + radius, 0, 0));
		westRod->setPosition3D(ax::Vec3(-(rodLength / 2 + radius), 0, 0));
		
		ball->addChild(northRod);
		ball->addChild(southRod);
		ball->addChild(eastRod);
		ball->addChild(westRod);
	}
	
	// Helper function to convert degrees to radians
	float degToRad(float degrees) {
		return degrees * (M_PI / 180.0f);
	}
	
	// Helper function to convert radians to degrees
	float radToDeg(float radians) {
		return radians * (180.0f / M_PI);
	}


	// This function returns a direction vector given an angle from the North (positive Z-axis)
	ax::Vec3 directionFromAngle(float angleInDegrees) {
		float rad = degToRad(angleInDegrees);
		return ax::Vec3(cos(rad), sin(rad), 0);
	}

	
	void addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, const ax::Vec3& direction) {
		auto pole1 = ax::MeshRenderer::create();
		auto pole2 = ax::MeshRenderer::create();
		
		auto poleMesh1 = createPole(distance, poleRadius, 20);
		auto poleMesh2 = createPole(distance, poleRadius, 20);

		pole1->addMesh(poleMesh1);
		pole2->addMesh(poleMesh2);
		
		poleMesh1->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		poleMesh2->setMaterial(poleMesh1->getMaterial());
		
		pole1->setTexture("gray.jpg");
		pole2->setTexture("gray.jpg");
		
		// Set position based on direction
		ax::Vec3 polePosition1 = direction * (innerRadius + distance / 2);
		ax::Vec3 polePosition2 = -direction * (innerRadius + distance / 2);

		pole1->setPosition3D(polePosition1);
		pole2->setPosition3D(polePosition2);
		
		// Rotate horizontally if direction is East-West (i.e., along Z-axis)
		if(direction == ax::Vec3(1, 0, 0)) {
			pole1->setRotation3D(ax::Vec3(0, 0, 90));  // Rotate 90 degrees about Y-axis
			pole2->setRotation3D(ax::Vec3(0, 0, 90));  // Rotate 90 degrees about Y-axis
		}


		node->addChild(pole1);
		node->addChild(pole2);
	}
	void addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, float angleFromNorth) {
		auto pole1 = ax::MeshRenderer::create();
		auto pole2 = ax::MeshRenderer::create();
		
		auto poleMesh1 = createPole(distance, poleRadius, 20);
		auto poleMesh2 = createPole(distance, poleRadius, 20);
		
		pole1->addMesh(poleMesh1);
		pole2->addMesh(poleMesh2);
		
		poleMesh1->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		poleMesh2->setMaterial(poleMesh1->getMaterial());
		
		pole1->setTexture("gray.jpg");
		pole2->setTexture("gray.jpg");
		
		ax::Vec3 direction = directionFromAngle(angleFromNorth);
		
		ax::Vec3 polePosition1 = direction * (innerRadius + distance / 2);
		ax::Vec3 polePosition2 = -direction * (innerRadius + distance / 2);
		
		pole1->setPosition3D(polePosition1);
		pole2->setPosition3D(polePosition2);
		
		if(fabs(direction.x) < fabs(direction.y)) {
			pole1->setRotation3D(ax::Vec3(0, 0, 45));  // Rotate 90 degrees about Y-axis
			pole2->setRotation3D(ax::Vec3(0, 0, 45));  // Rotate 90 degrees about Y-axis
		} else {
			pole1->setRotation3D(ax::Vec3(0, 0, 135));  // Rotate 90 degrees about Y-axis
			pole2->setRotation3D(ax::Vec3(0, 0, 135));  // Rotate 90 degrees about Y-axis
		}
		
		node->addChild(pole1);
		node->addChild(pole2);
	}
	
	Mesh* createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
		std::vector<float> positions;
		std::vector<float> normals;
		std::vector<float> texs;
		std::vector<unsigned short> indices;
		
		for (int m = 0; m <= majorSegments; m++) {
			float phi = 2.0 * M_PI * (float)m / (float)majorSegments;
			float cosPhi = cosf(phi);
			float sinPhi = sinf(phi);
			
			for (int n = 0; n <= minorSegments; n++) {
				float theta = 2.0 * M_PI * (float)n / (float)minorSegments;
				float cosTheta = cosf(theta);
				float sinTheta = sinf(theta);
				
				// Vertex positions
				float x = (majorRadius + minorRadius * cosTheta) * cosPhi;
				float y = (majorRadius + minorRadius * cosTheta) * sinPhi;
				float z = minorRadius * sinTheta;
				
				// Normals
				float nx = cosPhi * cosTheta;
				float ny = sinPhi * cosTheta;
				float nz = sinTheta;
				
				// Texture coordinates
				float u = (float)m / (float)majorSegments;
				float v = (float)n / (float)minorSegments;
				
				positions.push_back(x);
				positions.push_back(y);
				positions.push_back(z);
				
				normals.push_back(nx);
				normals.push_back(ny);
				normals.push_back(nz);
				
				texs.push_back(u);
				texs.push_back(v);
			}
		}
		
		// Indices
		for (int m = 0; m < majorSegments; m++) {
			for (int n = 0; n < minorSegments; n++) {
				int first = (m * (minorSegments + 1)) + n;
				int second = first + minorSegments + 1;
				int nextFirst = (m * (minorSegments + 1)) + ((n + 1) % (minorSegments + 1));
				int nextSecond = nextFirst + minorSegments + 1;
				
				indices.push_back(first);
				indices.push_back(second);
				indices.push_back(nextFirst);
				
				indices.push_back(second);
				indices.push_back(nextSecond);
				indices.push_back(nextFirst);
			}
		}
		
		IndexArray indexArray;
		for (auto index : indices) {
			indexArray.emplace_back(index);
		}
		
		return Mesh::create(positions, normals, texs, indexArray);
	}
	
	Mesh* createFlatDisk(float radius, float thickness, int majorSegments, int minorSegments) {
		// A flat disk is essentially a very flat torus
		return createTorus(radius + thickness / 2, thickness / 2, majorSegments, minorSegments);
	}
	
	ax::Vec3 rotateAroundAxis(const ax::Vec3& point, const ax::Vec3& axis, float angle) {
		if (axis == ax::Vec3(1, 0, 0)) { // Rotate around X
			return ax::Vec3(
							point.x,
							point.y * cosf(angle) - point.z * sinf(angle),
							point.y * sinf(angle) + point.z * cosf(angle)
							);
		} else if (axis == ax::Vec3(0, 1, 0)) { // Rotate around Y
			return ax::Vec3(
							point.x * cosf(angle) + point.z * sinf(angle),
							point.y,
							-point.x * sinf(angle) + point.z * cosf(angle)
							);
		} else if (axis == ax::Vec3(0, 0, 1)) { // Rotate around Z
			return ax::Vec3(
							point.x * cosf(angle) - point.y * sinf(angle),
							point.x * sinf(angle) + point.y * cosf(angle),
							point.z
							);
		} else {
			// No rotation
			return point;
		}
	}
	
	Vec3 calculateMagneticFieldAt(const Vec3& position) {
		Vec3 totalB(0, 0, 0);  // Initialize the total magnetic field to zero
		
		auto ironBall = dynamic_cast<MagneticBall*>(pinball);
		
		float ballMagneticMoment = ironBall->get_magnetization() * innerNode->getAngularSpeed();

		auto& magnets = alternator.getAttachedEntities();
		for (auto& attachedMagnet : magnets) {
			// Calculate vector pointing from magnet to the position
			Vec3 r = position - attachedMagnet.position;
			float distance = r.length();
					
			// Normalize the vector r
			r.normalize();
			
			// Calculate the magnetic moment (simplified) for the attached magnet
			float m = ballMagneticMoment * ironBall->calculate_volume();

			// Biot-Savart approximation for magnetic field due to a small magnet
			Vec3 B = (MU_0 / (4.0f * M_PI)) * (m / std::pow(distance, 3)) * r;
			
			// Sum up the contributions
			totalB += B;
		}

		
		return totalB;
	}
	
	float calculateFluxThroughCoil(const Vec3& B, float coilArea, int coilTurns) {
		return coilTurns * coilArea * B.length();
	}
	
	void calculateFlux(CoilEntity::AttachedEntity& coil, const Vec3& position) {
		auto ironBall = dynamic_cast<MagneticBall*>(pinball);
		float ballMagneticMoment = ironBall->get_magnetization() * ironBall->calculate_volume();
		
		Vec3 r = position - coil.position;
		float distance = r.length();
		r.normalize();
		
		// Magnetic field at the coil's position due to the ball
		Vec3 B = (MU_0 / (4.0f * M_PI)) * (ballMagneticMoment / std::pow(distance, 3)) * r;
		
		float flux = B.length() * coil.area; // assuming coil's face is perpendicular to B

		// Calculate the flux through the coil due to this magnetic field
		coil.flux = flux;
		
	}
	
	float calculateCoilEMF(const CoilEntity::AttachedEntity& coil, float delta) {
		// Change in flux
		float deltaPhi = coil.flux - coil.previousFlux;
		
		// EMF for this coil using Faraday's law
		return -coil.turns * (deltaPhi / delta);
	}
	
	void update(float dt) override {
		//global_delta = dt;
		
		middleMagnetSystem->update();
		innerMagnetSystem->update();
		outerCoilSystem->update();
		alternator.update();
		
		auto ironBall = dynamic_cast<MagneticBall*>(pinball);
		
		applyMagneticImpulse(global_delta);
		
		// Compute flux for all coils
		float totalEMF = 0.0f;
		auto& outerEntities = middleMagnetSystem->getAttachedEntities();
		auto& innerEntities = innerMagnetSystem->getAttachedEntities();

		auto& coils = alternator.getAttachedEntities();

		for (auto& coil : coils) {
			// Reset the coil's flux before summing from all entities
			coil.previousFlux = coil.flux;
			coil.flux = 0.0f;
			
			for (auto& entity : outerEntities) {
				// Compute flux for the current coil based on each entity's position
				calculateFlux(coil, entity.position);
			}
			
			// Compute the EMF for the coil after accounting for all entities
			totalEMF += calculateCoilEMF(coil, global_delta);
		}
//
//
//		for (auto& coil : coils) {
//			// Reset the coil's flux before summing from all entities
//			coil.previousFlux = coil.flux;
//			coil.flux = 0.0f;
//
//			for (auto& entity : innerEntities) {
//				// Compute flux for the current coil based on each entity's position
//				calculateFlux(coil, entity.position);
//			}
//
//			// Compute the EMF for the coil after accounting for all entities
//			totalEMF += calculateCoilEMF(coil, delta);
//		}


		// Compute total induced EMF in the alternator
		alternator.emf = totalEMF;
			

		outerCoilSystem->update(alternator.emf, global_delta);

	}
	const float EARTH_MAGNETIC_FIELD_STRENGTH = 50e-6; // 50 microteslas as an average value
	const float SIMULATION_SCALE = 1; // Arbitrary scale to make the force tangible in the simulation
	
	
	float calculateEffectiveArea(const Quaternion& rotation) {
		// Original dimensions of the innerNode
		const float length = 1.0f;  // Replace with actual value
		const float width = 1.0f;   // Replace with actual value
		float A_original = length * width;
		
		// Assuming the normal vector of the unrotated innerNode is (0, 0, 1)
		Vec3 unrotatedNormal(0, 0, 1);
		Vec3 rotatedNormal = rotation * unrotatedNormal;  // Apply quaternion rotation to the normal
		
		// Assuming the magnetic field direction is vertical (0, 0, 1)
		Vec3 magneticFieldDirection(0, 0, 1);
		
		// Compute the cosine of the angle between the rotated normal and the magnetic field direction
		float cosTheta = rotatedNormal.dot(magneticFieldDirection);
		
		return A_original * cosTheta;
	}
	
	void applyTorqueAndRotate(CustomNode* node, const Vec3& torque, float delta, Vec3 axis) {
		MagneticBall* magneticBall = dynamic_cast<MagneticBall*>(pinball);
		// Project torque onto the Z-axis
		float projectedTorqueMagnitude = torque.length();
		
		// 1. Get the inertia from the magnetic ball
		float inertia = magneticBall->calculate_inertia();
		
		// 2. Calculate angular acceleration from torque and inertia
		float angularAcceleration = projectedTorqueMagnitude / inertia;
		
		// 3. Update angular speed based on this acceleration
		float oldAngularSpeed = node->getAngularSpeed();
		float newAngularSpeed = oldAngularSpeed + angularAcceleration * delta;
				
		newAngularSpeed *= 0.1f;
		node->setAngularSpeed(newAngularSpeed);
		
		// 5. Calculate the rotation angle based on the updated angular speed
		float rotationAngle = newAngularSpeed * delta;
				
		// 6. Create the rotation and apply it (around Z-axis)
		Quaternion rotation;
		Quaternion::createFromAxisAngle(axis, rotationAngle, &rotation);
		Quaternion currentRotation = node->getRotationQuat();
		Quaternion newRotation = currentRotation * rotation;
		node->setRotationQuat(newRotation);
		
	}
	void applyMagneticImpulse(float delta) {
		auto ironBall = dynamic_cast<MagneticBall*>(pinball);

		// Combine magnetic forces
		auto forces = outerCoilSystem->combineFieldsOrForces();

		auto ironBallMagnets = innerMagnetSystem->getAttachedEntities();
		auto middleRingMagnets = middleMagnetSystem->getAttachedEntities();

		Vec3 ironBallTotalForce(0, 0, 0);
		Vec3 middleRingTotalForce(0, 0, 0);
		
		for (const auto& ironBallMagnet : ironBallMagnets) {
			for (const auto& middleRingMagnet : middleRingMagnets) {
				// Force on ironBallMagnet due to middleRingMagnet
				Vec3 forceOnIronBall =  innerMagnetSystem->calculateForceDueToMagnet(middleRingMagnet.position, ironBallMagnet.position, middleRingMagnet.polarity);
				ironBallTotalForce += forceOnIronBall;
				
				// Force on middleRingMagnet due to ironBallMagnet (opposite direction)
				Vec3 forceOnMiddleRing = -forceOnIronBall;
				middleRingTotalForce += forceOnMiddleRing;
			}
		}
		
		auto ironBallForces = ironBallTotalForce;
		auto middleForces = middleRingTotalForce;

		ironBallForces += forces;
		middleForces += forces;
			
		// Add Earth's magnetic force pointing north
//		Vec3 earthMagneticForce = Vec3(-EARTH_MAGNETIC_FIELD_STRENGTH, -EARTH_MAGNETIC_FIELD_STRENGTH, 0) * SIMULATION_SCALE;
		

		// Assuming that forces are acting on some lever arm distance r from the rotation axis
		float r = 1; // This value should be set based on your system
		
		Vec3 torqueInner = r * ironBallForces;
		applyTorqueAndRotate(innerNode, torqueInner, global_delta, Vec3(1, 0, 0));
		
		Vec3 torqueMiddle = r * middleForces;
		applyTorqueAndRotate(middleNode, torqueMiddle, global_delta, Vec3(0, 1, 0));
	}
	
	AlternatorSystem& getAlternatorSystem() { return alternator; }
	CoilSystem& getCoilSystem() { return *outerCoilSystem; }
	
	void applyCoilImpulse(float delta){
//		this->applyCoilForce(Vec3(0, 1, 0));
//		this->applyCoilForce(Vec3(0, -1, 0));
//
//		this->applyCoilForce(Vec3(0, 0, 1));
//		this->applyCoilForce(Vec3(0, 0, -1));
//
//		this->applyCoilForce(Vec3(1, 0, 0));
//		this->applyCoilForce(Vec3(-1, 0, 0));
	}
//
//	void applyCoilForce(Vec3 position){
//		// Initialize your CoilInductor somewhere (e.g., in the constructor)
//		CoilInductor coil(position, 0.01f, 320, 0.10f);
//
//		Vec3 ballPosition = dynamic_cast<MagneticBall*>(pinball)->getPosition3D();
//
//		Vec3 B = coil.computeMagneticField(ballPosition);
//		Vec3 gradB = coil.computeGradientB(ballPosition);
//
//		auto ball = dynamic_cast<MagneticBall*>(pinball);
//		if (ball) {
//			float forceValue = ball->getForceDueToMagneticField(B, gradB);
//			Vec3 forceDirection = gradB.getNormalized() * forceValue;
//
//			auto physicsObject = ball->getPhysics3DRigidBody();
//			if (physicsObject) {
//				physicsObject->applyForce(forceDirection, {});
//			}
//		}
//	}
//
//
	CustomNode* outerNode;
	CustomNode* middleNode;
	CustomNode* innerNode;
	
	ax::Mesh* outerRing;
	ax::Mesh* middleRing;
	ax::Mesh* innerRing;
	ax::Node* pinball;
	
	std::unique_ptr<CoilSystem> outerCoilSystem =
	std::make_unique<CoilSystem>(1.5f,
								 1.0f, // resistance
								 1.5, // current
								 144); // turns
	AlternatorSystem alternator = AlternatorSystem(0.01f, 144);
	
	std::unique_ptr<MagnetSystem> middleMagnetSystem = std::make_unique<MagnetSystem>();
	
	std::unique_ptr<MagnetSystem> innerMagnetSystem = std::make_unique<MagnetSystem>();

	float baseDistanceOffset = 0.07f; // Base offset for the distance
	
	float innerRingRadius = 0.05f; // Base offset for the distance
	float middleRingRadius = 0.06f; // Base offset for the distance
	float outerRingRadius = 0.12f; // Base offset for the distance

public:
	CREATE_FUNC(MaritimeGimbal3D);
	
	bool init() override {
		if (!ax::Node::init()) return false;
		
		float baseThicknessOffset = 0.0012f;  // Base thickness offset for the gimbal rings
		
		// Create the three gimbal rings using the given parameters
		innerRing = createFlatDisk(innerRingRadius, baseThicknessOffset, 40, 20);
		middleRing = createFlatDisk(middleRingRadius, baseThicknessOffset * (2.0f / 3.0f), 40, 20);
		outerRing = createFlatDisk(outerRingRadius, baseThicknessOffset, 40, 20);

		outerNode = CustomNode::create();
		middleNode = CustomNode::create();
		innerNode = CustomNode::create();
		
		outerNode->addMesh(outerRing);
		middleNode->addMesh(middleRing);
		innerNode->addMesh(innerRing);
		
		outerRing->setTexture("gold.jpg");
		middleRing->setTexture("gold.jpg");
		innerRing->setTexture("gold.jpg");
		
		// Outer Node Magnets with NORTH polarity
		outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::NORTH, MagnetPolarity::NORTH);
		outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH);
		
		outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::EAST, MagnetPolarity::NORTH);
		outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::WEST, MagnetPolarity::SOUTH);
		

		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::NORTHEAST, MagnetPolarity::SOUTH);
		
		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::SOUTHEAST, MagnetPolarity::SOUTH);

		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::NORTHWEST, MagnetPolarity::SOUTH);

		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::SOUTHWEST, MagnetPolarity::SOUTH);

		
		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::FRONT, MagnetPolarity::SOUTH);

		
		alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::BACK, MagnetPolarity::SOUTH);


		middleMagnetSystem->attachToDisk(middleNode, middleRingRadius + 0.0016f, MagnetDirection::WEST, MagnetPolarity::SOUTH);
		middleMagnetSystem->attachToDisk(middleNode, (baseDistanceOffset - 0.01f) + 0.0016f, MagnetDirection::EAST, MagnetPolarity::NORTH);
		
		innerMagnetSystem->attachToDisk(innerNode, innerRingRadius + 0.0016f, MagnetDirection::NORTH, MagnetPolarity::NORTH);
		innerMagnetSystem->attachToDisk(innerNode, innerRingRadius + 0.0016f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH);

		
		this->outerNode->setMaterial(MeshMaterial::createBuiltInMaterial(MeshMaterial::MaterialType::UNLIT, false));
		this->middleNode->setMaterial(this->outerNode->getMaterial());
		this->innerNode->setMaterial(this->outerNode->getMaterial());
		
		this->addChild(outerNode);
		outerNode->addChild(middleNode);
		middleNode->addChild(innerNode);
		
		this->addPoles(outerNode, middleRingRadius, (outerRingRadius - middleRingRadius), 0.001f, ax::Vec3(0, 1, 0));  // Poles for the outer ring, along Y-axis
		this->addPoles(middleNode, innerRingRadius, (middleRingRadius - innerRingRadius), 0.001f, ax::Vec3(1, 0, 0)); // Poles for the middle ring, along Z-axis
		// Adding poles:
		this->addPoles(innerNode, 0.04f, (innerRingRadius - 0.04f), 0.001f, 45);  // NE
		this->addPoles(innerNode, 0.04f, (innerRingRadius - 0.04f), 0.001f, 135); // SE
		this->scheduleUpdate();

		
		return true;
	}
	
	void attachPinball(Scene* scene) {
		
		pinball = IronBall::create(0.04f);
		
		//scene->getPhysics3DWorld()->addPhysics3DObject(dynamic_cast<MagneticBall*>(pinball)->getPhysics3DRigidBody());
		
		innerNode->addChild(pinball);

		middleMagnetSystem->update();
		innerMagnetSystem->update();
		outerCoilSystem->update();
		alternator.update();
		
		
		// Compute flux for all coils
		float totalEMF = 0.0f;
		auto& outerEntities = middleMagnetSystem->getAttachedEntities();
		auto& innerEntities = innerMagnetSystem->getAttachedEntities();
		
		auto& coils = alternator.getAttachedEntities();
		
		for (auto& coil : coils) {
			// Reset the coil's flux before summing from all entities
			coil.previousFlux = coil.flux;
			coil.flux = 0.0f;
			
			for (auto& entity : outerEntities) {
				// Compute flux for the current coil based on each entity's position
				calculateFlux(coil, entity.position);
			}

			coil.previousFlux = coil.flux;

		}
	}
	
};


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
	static float guiEMF = 0;
	static float guiMeasure = 0;
	static float peakEMF = 0;
	static float accumulatedEMF = 0;

	deltaCounter += deltaTime;
	guiCounter += deltaTime;
	
	float inducedEMF = abs(magnos->getAlternatorSystem().emf);
	
	if(!magnos->getCoilSystem().calibrating() || magnos->getCoilSystem().adapting()){
		accumulatedEMF = magnos->getCoilSystem().lastAccumulatedEMF;
	}
	
	if(guiCounter >= 1){
		guiEMF = accumulatedEMF;
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
	ImGui::Text("Measured Voltage Per Second=%.4f", guiEMF);

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

