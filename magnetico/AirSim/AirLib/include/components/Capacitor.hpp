#pragma once

class Capacitor {
private:
    float voltage; // Current voltage across the capacitor
    float capacity; // Maximum capacity of the capacitor
    
public:
    Capacitor(float initialCapacity);
    
    // Discharge the capacitor by a given amount
    void discharge(float amount);
    
    // Charge the capacitor by a given amount, up to its capacity
    void charge(float amount);
    
    float getVoltage() const;

    float getCapacity() const;
};
