#include "components/VoltageController.hpp"

VoltageController::VoltageController(size_t size, float target, float capacitorCapacity, bool recycleVoltage)
	: filter(size), capacitor(capacitorCapacity), targetVoltage(target), accumulatedRemainder(0.0f), onCapacitorCharged(nullptr), recycle(recycleVoltage) {}

	float VoltageController::controlVoltage(float inputValue) {
		// Calculate the remainder from the previous step
		if (inputValue <= targetVoltage) {
			return inputValue;
		}
		
		// Adjust the voltage to be no more than targetVoltage
		if(recycle){
			float adjustedValue = inputValue;
			if (adjustedValue > targetVoltage) {
				chargeCapacitor(adjustedValue - targetVoltage);
				adjustedValue = targetVoltage;
			}
			
			// Apply filtering
			float filteredValue = filter.filter(adjustedValue);
						
			return filteredValue;

		} else {
			float adjustedValue = inputValue;
			if (adjustedValue > targetVoltage) {
				adjustedValue = targetVoltage;
			}
			
			// Apply filtering
			float filteredValue = filter.filter(adjustedValue);
			
			return filteredValue;

		}
	}
	
	void VoltageController::onVoltageFiltered(){
		if (capacitor.getVoltage() >= capacitor.getCapacity()) {
			dischargeCapacitor();
		}
	}
	
	// Charge the capacitor by a given amount and execute the onCapacitorCharged callback
	void VoltageController::chargeCapacitor(float amount) {
		capacitor.charge(amount);
	}
	
	// Discharge the capacitor completely and execute the onCapacitorCharged callback
	void VoltageController::dischargeCapacitor() {
		float currentVoltage = capacitor.getVoltage();
		capacitor.discharge(currentVoltage);
		if (onCapacitorCharged) {
			onCapacitorCharged(currentVoltage);
		}
	}
	
	// Get the current voltage of the capacitor
	float VoltageController::getCapacitorVoltage() const {
		return capacitor.getVoltage();
	}
	
	float VoltageController::consumeFromCapacitor(float amount) {
		capacitor.discharge(amount);
		return amount;
	}

	// Set the callback for when the capacitor is fully charged
	void VoltageController::setOnCapacitorCharged(std::function<void(float)> callback) {
		onCapacitorCharged = callback;
	}
