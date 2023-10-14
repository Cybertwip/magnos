#include "components/Battery.hpp"

RechargeableBattery::RechargeableBattery(float initialVoltage, float initialCapacity, float maxCapacity, float chargeEfficiency, float dischargeEfficiency) :
voltage(initialVoltage), capacity(initialCapacity), maxCapacity(maxCapacity), chargeEfficiency(chargeEfficiency), dischargeEfficiency(dischargeEfficiency) {}

// Charge the battery with a specified current (in Amperes) for a given time (in seconds)
void RechargeableBattery::charge(float current, float time) {
	float chargeAmount = current * time * chargeEfficiency;
	// Ensure that the capacity doesn't exceed the maximum capacity
	if (capacity + chargeAmount > maxCapacity) {
		capacity = maxCapacity;
	} else {
		capacity += chargeAmount;
	}
}

// Discharge the battery with a specified current (in Amperes) for a given time (in seconds)
void RechargeableBattery::discharge(float current, float time) {
	float dischargeAmount = current * time * dischargeEfficiency;
	
	// Ensure that the capacity doesn't become negative
	if (capacity - dischargeAmount < 0.0f) {
		capacity = 0.0f;
	} else {
		capacity -= dischargeAmount;
	}
}

// Get the current battery voltage
float RechargeableBattery::getVoltage() const {
	return voltage;
}

// Get the remaining capacity of the battery
float RechargeableBattery::getCapacity() const {
	return capacity;
}

