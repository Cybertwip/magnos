#pragma once

#include "components/Coil.hpp"
#include "components/PIDController.hpp"
#include "components/VoltageController.hpp"
#include "components/entities/CoilEntity.hpp"

namespace mlpack{
	class LinearRegression;
}

namespace Settings {
	const float fps = 240;
	const float fixed_update = 120;
	const long global_delta = 1.0f / fixed_update * 1000;
	const float fixed_delta = 1.0f / fixed_update;

	const float battery_voltage = 600;

	const int data_collection_bin_size = 1 /* kb */ * 128 * 1024;

	const float error_trial = 0.25f;

	const float desired_base_voltage = 4.5f;
	const float desired_laser_voltage = 5.0f;

	const int number_of_gimbals = 8;

	const int data_collection_mode_cycles = (72 * 2) / number_of_gimbals;

	const float desired_capacitor_voltage = 4.5f;

	const float coil_capacitor_voltage = 35.0f;

	const float circuit_resistance = 120.0f;

	const bool enable_lasers = true;
	const int number_of_lasers = 6;
}

class CoilSystem : public CoilEntity {
public:
	float coilResistance;    // Resistance of the coil in Ohms
	float maxCurrent;
	float emf;
	
	float current = 0;
	Capacitor accumulator = Capacitor(Settings::coil_capacitor_voltage);

	float turns;
	
	std::vector<std::shared_ptr<Coil>> coils;
	
public:
	CoilSystem(const std::string& writablePath, int id, float voltage, float resistance, float current, float coilTurns);
	
	~CoilSystem();
	
	float withdrawPower(float power);
	float testWithdrawPower(float power);
	float storePower(float power);
	
	void recalibrate();
	void resetAccumulators();
	void saveDataToBinary(const std::string& filename);
	bool loadDataAndTrainModel(const std::string& filename);
	void setCurrentFromVoltage(float voltage);
	msr::airlib::Vector3r computeMagneticField(CoilEntity::AttachedEntity& coil, const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& point, MagnetPolarity polarity) const;
	msr::airlib::Vector3r combineFieldsOrForces(const msr::airlib::Vector3r& origin) override;

	bool calibrating();
	bool collecting();

	void scheduleCollection();
	
	void setDesignedEMFPerSecond(float desiredEMF);
	void setOnVoltagePeakCallback(std::function<void(int, float)> onVoltagePeak);

	void adjustCurrentBasedOn(float dt);
	std::shared_ptr<msr::airlib::Node> attach(float radius, MagnetDirection direction, MagnetPolarity polarity);

private:
		
	int coil_id;
	bool data_collection_mode = false;
	bool schedule_data_collection_mode = false;
	bool schedule_recalibration_for_collection = false;

	
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

	std::unique_ptr<mlpack::LinearRegression> mlModel;
	
	bool hasML = false;


	bool calibration = true;

	const int numberOfCoils = 6;
	const float totalResistance = Settings::circuit_resistance;
	float baseAccumulatedEMF = 0.0f;
	long long accumulationTime = 0;
	float recycledEMF = 0;
	

	PIDController pidCurrent = PIDController(1.0f, 0, 0);

	VoltageController filterBase;
	VoltageController filterIncrease;
	
	MovingAverageFilter totalEMFAverageFilter = MovingAverageFilter(240);
	MovingAverageFilter baseEMFAverageFilter = MovingAverageFilter(240);
	
	float designedEMFPerSecond;
	std::function<void(int, float)> onVoltagePeak = nullptr;
	
	std::string writablePath;
public:
	float lastAccumulatedEMF = 0.0f;
	float lastBaseAccumulatedEMF = 0.0f;
	float lastRecycledEMF = 0.0f;
	
	void update();
	void update(float measuredEMF, float delta);
};
