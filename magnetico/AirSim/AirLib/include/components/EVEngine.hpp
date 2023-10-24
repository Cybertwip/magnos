#pragma once

#include "components/entities/Node.hpp"

#include "common/Common.hpp"

class Laser;
class Magnos;
class RechargeableBattery;

struct EVFeedback {
	std::string status;
	float peakEMF;
	float baseEMF;
	float EMF;
};

class EVEngine : public msr::airlib::Node {
public:
	EVEngine(float max_engine_voltage = EVEngine::max_voltage);
	
	void init(const std::string& writablePath);
	void update(float dt) override;

	float getBatteryVoltage() const;
	const EVFeedback& getMagnosFeedback() const;

	float accelerate(float throttle);
	void decelerate();
	
	bool isCalibrating();
	
	void setEngineConsumption(float voltage);
	float getEngineConsumption() const;
	//@TODO REMOVE
	std::vector<std::shared_ptr<Magnos>> getGimbals() const {
		return this->gimbals_;
	}

	std::vector<std::shared_ptr<Laser>> getLasers() const {
		return this->lasers_;
	}

	std::shared_ptr<RechargeableBattery> getBattery() const {
		return this->battery_;
	}

public:
	static float max_voltage;
	
private:
	std::vector<std::shared_ptr<Magnos>> gimbals_;
	std::vector<std::shared_ptr<Laser>> lasers_;
	std::shared_ptr<RechargeableBattery> battery_;
	bool accelerating_;
	EVFeedback feedback_;
	
	float max_engine_voltage_;
	
	float engine_consumption_;
};

