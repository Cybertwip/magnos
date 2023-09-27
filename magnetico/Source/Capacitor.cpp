#include "Capacitor.h"

Capacitor::Capacitor(float initialCapacity) : voltage(0.0f), capacity(initialCapacity) {
    
}

// Discharge the capacitor by a given amount
void Capacitor::discharge(float amount) {
    voltage -= amount;
    if (voltage < 0.0f) {
        voltage = 0.0f;
    }
}

// Charge the capacitor by a given amount, up to its capacity
void Capacitor::charge(float amount) {
    voltage += amount;
    if (voltage > capacity) {
        voltage = capacity;
    }
}

float Capacitor::getVoltage() const {
    return voltage;
}

float Capacitor::getCapacity() const {
    return capacity;
}