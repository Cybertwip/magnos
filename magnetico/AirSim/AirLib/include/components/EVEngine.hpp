#pragma once

#include "components/entities/Node.hpp"

#include "common/Common.hpp"

class Magnos;
class RechargeableBattery;

struct EVFeedback {
	std::string status;
	float peakEMF;
	float baseEMF;
	float EMF;
};

class EVEngine : public Node {
public:
	EVEngine();
	
	void init();
	void update(float dt) override;

	float getBatteryVoltage() const;
	const EVFeedback& getMagnosFeedback() const;

	float accelerate(float throttle);
	void decelerate();
	
	
	//@TODO REMOVE
	std::vector<std::shared_ptr<Magnos>> getGimbals() const {
		return this->gimbals_;
	}
	
	std::shared_ptr<RechargeableBattery> getBattery() const {
		return this->battery_;
	}

private:
	std::vector<std::shared_ptr<Magnos>> gimbals_;
	std::shared_ptr<RechargeableBattery> battery_;

	bool accelerating_;

	EVFeedback feedback_;
};

