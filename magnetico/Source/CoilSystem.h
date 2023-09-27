#pragma once

#include "PIDController.h"
#include "VoltageController.h"
#include "CoilEntity.h"
#include "Coil.h"

#include "Settings.h"

#include <axmol.h>

#include <mlpack.hpp>

class CoilSystem : public CoilEntity {
public:
	float coilResistance;    // Resistance of the coil in Ohms
	float maxCurrent;
	float emf;
	
	float current = 0;
	float turns;
	
	std::vector<Coil*> coils;
	
public:
	CoilSystem(float voltage, float resistance, float current, float coilTurns);
	
	~CoilSystem();

	// Function to retrain model
	void retrainModel();
	void saveDataToBinary(const std::string& filename);
	void savePIDToBinary(const std::string& filename);
	bool loadDataAndTrainModel(const std::string& filename);
	bool loadPID(const std::string& filename);
	void setCurrentFromVoltage(float voltage);	
	ax::Vec3 computeMagneticField(AttachedEntity& coil, const ax::Vec3& point, MagnetPolarity polarity) const;
	ax::Vec3 combineFieldsOrForces() override;

	bool calibrating();
	bool adapting();
	void adjustCurrentBasedOn(float dt);
	void attachToDisk(ax::Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) override;	

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

	std::unique_ptr<mlpack::regression::LinearRegression> mlModel;
	
	bool hasML = false;


	bool calibration = true;
	bool adaptive = adaptive_calibration;

	float desiredEMFPerSecond = desired_voltage_increase_per_second;
	const int numberOfCoils = 1;
	const float totalResistance = coilResistance * numberOfCoils;
	float accumulatedEMF = 0.0f;
	float baseAccumulatedEMF = 0.0f;
	float accumulationTime = 0.0f;
	float guiBaseAccumulatedEMF = 0.0f;
	float guiAccumulatedEMF = 0.0f;
	float recycledEMF = 0;
	float guiAccumulationTime = 0;
	
	double setPoint = 3.0f; // Desired value
	double processVariable = 0; // Current value
	double controllerOutput = 0; // PID output
	
	long long previousTime = 0;
	long long nowTime = 0;

	PIDController pidCurrent = PIDController(1.0f, 0, 0);
	bool isCalibratingUpwards = true; // A flag to determine the calibration direction. Initialize as true if you start by calibrating upwards.

	VoltageController filterBase = VoltageController(4, desired_base_voltage, desired_base_voltage, false);
	VoltageController filterIncrease = VoltageController(4, desired_voltage_increase_per_second, desired_voltage_increase_per_second, true);

public:
	float lastAccumulatedEMF = 0.0f;
	float lastBaseAccumulatedEMF = 0.0f;
	float lastRecycledEMF = 0.0f;
	
	void update();
	void update(float measuredEMF, float delta);
};
