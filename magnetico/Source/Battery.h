#pragma once

class RechargeableBattery {
private:
	float voltage; // Battery voltage in volts
	float capacity; // Battery capacity in ampere-hours (Ah)
	float maxCapacity; // Maximum capacity of the battery in Ah
	float chargeEfficiency; // Charging efficiency coefficient (0 to 1)
	float dischargeEfficiency; // Discharging efficiency coefficient (0 to 1)
	
public:
	RechargeableBattery(float initialVoltage, float initialCapacity, float maxCapacity, float chargeEfficiency, float dischargeEfficiency);

	// Charge the battery with a specified current (in Amperes) for a given time (in seconds)
	void charge(float current, float time);
	
	// Discharge the battery with a specified current (in Amperes) for a given time (in seconds)
	void discharge(float current, float time);
	
	// Get the current battery voltage
	float getVoltage() const;
	
	// Get the remaining capacity of the battery
	float getCapacity() const;
};
