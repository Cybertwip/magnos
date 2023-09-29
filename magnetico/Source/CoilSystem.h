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
	
	void recalibrate();
	void resetAccumulators();
	void saveDataToBinary(const std::string& filename);
	bool loadDataAndTrainModel(const std::string& filename);
	void setCurrentFromVoltage(float voltage);
	ax::Vec3 computeMagneticField(AttachedEntity& coil, const ax::Vec3& point, MagnetPolarity polarity) const;
	ax::Vec3 combineFieldsOrForces() override;

	bool calibrating();
	void setDesignedEMFPerSecond(float desiredEMF);
	void setOnVoltagePeakCallback(std::function<void(float)> onVoltagePeak);

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

	float desiredEMFPerSecond = Settings::desired_target_voltage;
	const int numberOfCoils = 1;
	const float totalResistance = coilResistance * numberOfCoils;
	float accumulatedEMF = 0.0f;
	float baseAccumulatedEMF = 0.0f;
	float accumulationTime = 0.0f;
	float guiBaseAccumulatedEMF = 0.0f;
	float recycledEMF = 0;
	
	double setPoint = 3.0f; // Desired value
	double processVariable = 0; // Current value
	double controllerOutput = 0; // PID output

	PIDController pidCurrent = PIDController(1.0f, 0, 0);
	bool isCalibratingUpwards = true; // A flag to determine the calibration direction. Initialize as true if you start by calibrating upwards.

	VoltageController filterBase;
	VoltageController filterIncrease;
	
	float designedEMFPerSecond;
	std::function<void(float)> onVoltagePeak = nullptr;
	
public:
	float lastAccumulatedEMF = 0.0f;
	float lastBaseAccumulatedEMF = 0.0f;
	float lastRecycledEMF = 0.0f;
	
	void update();
	void update(float measuredEMF, float delta);
};
