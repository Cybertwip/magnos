#pragma once

#include "components/entities/Node.hpp"

#include "common/Common.hpp"

class Magnos;
class RechargeableBattery;

class EVEngine : public Node {
public:
	EVEngine();
	
	void init();
	void update();

	float getVoltage() const;

	void accelerate(float throttle);
	void decelerate();

private:
	std::vector<std::shared_ptr<Magnos>> gimbals_;
	std::shared_ptr<RechargeableBattery> battery_;

	bool accelerating_;
};

