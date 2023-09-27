#pragma once

#include "MovingAverageFilter.h"
#include "Capacitor.h"

#include <functional>

class VoltageController {
private:
	MovingAverageFilter filter;
	Capacitor capacitor;
	float targetVoltage;
	float accumulatedRemainder;
	bool recycle;
	
public:
	VoltageController(size_t size, float target, float capacitorCapacity, bool recycleVoltage);
	float controlVoltage(float inputValue);
	
	void onVoltageFiltered();
	
	// Charge the capacitor by a given amount and execute the onCapacitorCharged callback
	void chargeCapacitor(float amount);
	
	// Discharge the capacitor completely and execute the onCapacitorCharged callback
	void dischargeCapacitor();
	
	// Get the current voltage of the capacitor
	float getCapacitorVoltage() const ;
	
	// Set the callback for when the capacitor is fully charged
	void setOnCapacitorCharged(std::function<void(float)> callback);

private:
	std::function<void(float)> onCapacitorCharged;
};
