#include <Arduino.h>
#include <ArduinoSTL.h>

#include <Stepper.h>

struct DataPoint {
    double emfError;
    double desiredCurrent;
    double currentAdjustment;
    double finalCurrent;
};

class LinearRegressionModel {
private:
    double intercept;
    double slope;

public:
    LinearRegressionModel() : intercept(0), slope(0) {}

    void train(const std::vector<DataPoint>& data) {
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = data.size();

        if (n == 0) return;  // Prevent division by zero if no data is present

        for (const auto& point : data) {
            sum_x += point.desiredCurrent;
            sum_y += point.finalCurrent;
            sum_xx += point.desiredCurrent * point.desiredCurrent;
            sum_xy += point.desiredCurrent * point.finalCurrent;
        }

        double denominator = n * sum_xx - sum_x * sum_x;
        if (denominator != 0) {
            slope = (n * sum_xy - sum_x * sum_y) / denominator;
            intercept = (sum_y - slope * sum_x) / n;
        } else {
            slope = 0;  // Consider setting to NaN or handle as an error
            intercept = 0;
        }
    }

    double predict(double desiredCurrent) {
        return intercept + slope * desiredCurrent;
    }
};

class PIDController {
private:
    float kp, ki, kd;
    float integral = 0.0;
    float prev_error = 0.0;
    bool autoTuning = false;
    float relayHigh, relayLow;
    long long prevSwitchTime = 0, oscillationStartTime = 0;
    int oscillationsCount = 0;
    float period = 0.0;
    int calibrationSteps = 12;

public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    void startAutoTuning(float relayHigh, float relayLow) {
        if(!autoTuning){
          autoTuning = true;
        }
        this->relayHigh = relayHigh;
        this->relayLow = relayLow;
        // Do not reset oscillationsCount and period here if resuming
        if (oscillationsCount == 0) {
            period = 0.0;
            prevSwitchTime = millis(); // Re-initialize timing only if starting fresh
        }
    }

    bool calibrating() { return autoTuning; }

    void calibrate(float error, long long currentTime) {
        if (!autoTuning) return;

        bool shouldToggle = error != prev_error;
        long long timeDelta = currentTime - prevSwitchTime;

        if (shouldToggle) {
            if (oscillationsCount > 0) {
                period += timeDelta;
            }
            prevSwitchTime = currentTime;
            oscillationsCount++;
        }

        prev_error = error;

        if (oscillationsCount >= calibrationSteps) {
            autoTuning = false;
            period /= (oscillationsCount - 1);
            kp = 0.5 * (relayHigh - relayLow) / period; // More aggressive proportional gain
            ki = 8 * kp / period;
            kd = kp * period / 4;

            // Reset for possible future re-calibration
            oscillationsCount = 0;

        }
    }

    float compute(float error, float dt) {
        float p = kp * error;
        integral += error * dt;
        float d = kd * (error - prev_error) / dt;
        float output = p + ki * integral + d;
        prev_error = error;
        return output;
    }
};

class MovingAverageFilter {
private:
    std::vector<float> buffer;
    size_t windowSize;
    
public:
    MovingAverageFilter(size_t size);
    
    float filter(float inputValue);
};

MovingAverageFilter::MovingAverageFilter(size_t size) : windowSize(size) {}

float MovingAverageFilter::filter(float inputValue) {
    buffer.push_back(inputValue);
    if (buffer.size() > windowSize) {
        buffer.erase(buffer.begin());
    }
    
    float sum = 0.0f;
    for (const float& value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}


// Define constants and create PIDController instances
const int coilAPin = 2;
const int coilBPin = 3;
const int receiverPinA = A4;
const int maxFrequencyA = 4000;
const int maxFrequencyB = 4300;
const float maxCurrent = 255;
const float maxReading = 1023;

const int numTrainSamples = 10;

PIDController frequencyPIDA(0.1, 0.1, 0.05); // Increased gains for more aggressive response
PIDController frequencyPIDB(0.1, 0.1, 0.4); // Increased gains for more aggressive response
PIDController amplitudePIDA(0.1, 0.1, 0.1); // Reasonable gains for amplitude control
PIDController amplitudePIDB(0.15, 0.1, 2); // Adjusted gains for amplitude control


bool tuningFrequencyA = true;
bool tuningAmplitudeA = true;

long long lastTimeA = 0;
long long lastTimeB = 0;

long long lastTimeOscillation = 0;

bool stateA = false;
bool stateB = false;

float thisFrequencyA = 0;
float thisFrequencyB = 0;
float thisAmplitudeA = 0;
float thisAmplitudeB = 0;

const float targetCurrent = maxReading * 0.85; // Example: 75% of maximum reading

LinearRegressionModel mlFrequencyA;
LinearRegressionModel mlFrequencyB;
LinearRegressionModel mlAmplitudeA;
LinearRegressionModel mlAmplitudeB;

std::vector<DataPoint> dataFrequencyA, dataFrequencyB;
std::vector<DataPoint> dataAmplitudeA, dataAmplitudeB;

MovingAverageFilter softFilter(32);
MovingAverageFilter softFilterFreqA(8);
MovingAverageFilter softFilterFreqB(8);

// Define the clamp function
float clamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}

float interpolate(float current, float target, float step) {
    if (abs(current - target) < step) return target; // Close enough to target
    if (current < target) return current + step;     // Increment current
    if (current > target) return current - step;     // Decrement current
    return current;                                  // Default case (should not hit)
}


// Number of steps per revolution of the internal gear shaft
int stepsPerRevolution = 2048; // Change this depending on your motor's specification

// Pins connected from the motor to the driver IN1, IN2
int motorPins[4] = {8, 9, 10, 11}; // Only using 2 pins for simplicity

// Initialize the stepper library on pins 8 and 9
Stepper myStepper(stepsPerRevolution, motorPins[0], motorPins[1], motorPins[2], motorPins[3]);


void setup() {

    pinMode(coilAPin, OUTPUT);
    pinMode(coilBPin, OUTPUT);
    pinMode(receiverPinA, INPUT);
    Serial.begin(230400); // Set baud rate to 230400 bps

    // Start auto-tuning for frequency and amplitude PID controllers
    frequencyPIDA.startAutoTuning(maxFrequencyA, 0);
    frequencyPIDB.startAutoTuning(maxFrequencyB, 0);
    amplitudePIDA.startAutoTuning(maxReading, 0);
    amplitudePIDB.startAutoTuning(maxReading, 0);

    lastTimeA = millis();
    lastTimeB = millis();
    lastTimeOscillation = millis();

    myStepper.setSpeed(100);
    pinMode(motorPins[0], OUTPUT);
    pinMode(motorPins[1], OUTPUT);

}

void loop() {
    long long currentTime = millis();
    static long long lastTime = currentTime;
    long long delta = (currentTime - lastTime); // Calculate delta time in milliseconds
    float dt = (delta) / 1000.0; // Calculate delta time in seconds
    lastTime = currentTime;

    int currentA = softFilter.filter(analogRead(receiverPinA));

    if(currentA >= 900){
      return;
    }
    
    float emfError = (targetCurrent - currentA) / maxReading; // Adjusted error calculation
    float desiredCurrent = ((1.0 - emfError) * maxCurrent);

    frequencyPIDA.startAutoTuning(maxFrequencyA, 0);
    frequencyPIDB.startAutoTuning(maxFrequencyB, 0);
    amplitudePIDA.startAutoTuning(maxReading, 0);
    amplitudePIDB.startAutoTuning(maxReading, 0);

    // Compute PID outputs for frequency and amplitude control for coils A and B
    if(delta > 0){

      float frequencyAdjustmentA;
      float frequencyAdjustmentB;
      float amplitudeAdjustmentA;
      float amplitudeAdjustmentB;
      if(dataFrequencyA.size() < numTrainSamples){
        frequencyAdjustmentA = frequencyPIDA.compute(emfError, dt);
        frequencyAdjustmentB = frequencyPIDB.compute(emfError, dt);
        amplitudeAdjustmentA = amplitudePIDA.compute(desiredCurrent, dt);
        amplitudeAdjustmentB = amplitudePIDB.compute(desiredCurrent, dt);
      } else {
        frequencyAdjustmentA = mlFrequencyA.predict(emfError);
        frequencyAdjustmentB = mlFrequencyB.predict(emfError);
        amplitudeAdjustmentA = mlAmplitudeA.predict(desiredCurrent);
        amplitudeAdjustmentB = mlAmplitudeB.predict(desiredCurrent);
      }

      // Smoothly adjust amplitude by adding the adjustment to the current amplitude
      thisAmplitudeA = interpolate(thisAmplitudeA, thisAmplitudeA + amplitudeAdjustmentA, 5);
      thisAmplitudeB = interpolate(thisAmplitudeB, thisAmplitudeB + amplitudeAdjustmentB, 5);

      // Clamp the adjusted amplitude within the valid range
      thisAmplitudeA = clamp(thisAmplitudeA, 0.0f, maxCurrent);
      thisAmplitudeB = clamp(thisAmplitudeB, 0.0f, maxCurrent);

      // Map the amplitude values to the range [0, 255]
      thisAmplitudeA = map(thisAmplitudeA, 0, maxCurrent, 0, 255);
      thisAmplitudeB = map(thisAmplitudeB, 0, maxCurrent, 0, 255);

      thisFrequencyA = interpolate(thisFrequencyA, thisFrequencyA + frequencyAdjustmentA, 5);
      thisFrequencyB = interpolate(thisFrequencyB, thisFrequencyB + frequencyAdjustmentB, 5);


      thisFrequencyA = clamp(thisFrequencyA, 0.0f, maxFrequencyA);
      thisFrequencyB = clamp(thisFrequencyB, 0.0f, maxFrequencyB);


      if(dataFrequencyA.size() < numTrainSamples){
        DataPoint frequencyDataA;
        DataPoint frequencyDataB;
        DataPoint amplitudeDataA;
        DataPoint amplitudeDataB;

        frequencyDataA.emfError = emfError;
        frequencyDataB.emfError = emfError;
        amplitudeDataA.emfError = emfError;
        amplitudeDataB.emfError = emfError;

        frequencyDataA.desiredCurrent = emfError;
        frequencyDataB.desiredCurrent = emfError;
        amplitudeDataA.desiredCurrent = desiredCurrent;
        amplitudeDataB.desiredCurrent = desiredCurrent;

        frequencyDataA.currentAdjustment = frequencyAdjustmentA;
        frequencyDataB.currentAdjustment = frequencyAdjustmentB;
        amplitudeDataA.currentAdjustment = amplitudeAdjustmentA;
        amplitudeDataB.currentAdjustment = amplitudeAdjustmentB;

        frequencyDataA.finalCurrent = currentA;
        frequencyDataB.finalCurrent = currentA;
        amplitudeDataA.finalCurrent = currentA;
        amplitudeDataB.finalCurrent = currentA;

        // Add to vectors
        if(dataFrequencyA.size() < numTrainSamples){
          dataFrequencyA.push_back(frequencyDataA);
          dataFrequencyB.push_back(frequencyDataB);
          dataAmplitudeA.push_back(amplitudeDataA);
          dataAmplitudeB.push_back(amplitudeDataB);

        } 

        if (dataFrequencyA.size() >= numTrainSamples) {
            mlFrequencyA.train(dataFrequencyA);
            dataFrequencyA.clear(); // Clear after training
        }

        // Similarly for other models
        if (dataFrequencyB.size() >= numTrainSamples) {
            mlFrequencyB.train(dataFrequencyB);
            dataFrequencyB.clear();
        }

        if (dataAmplitudeA.size() >= numTrainSamples) {
            mlAmplitudeA.train(dataAmplitudeA);
            dataAmplitudeA.clear();
        }

        if (dataAmplitudeB.size() >= numTrainSamples) {
            mlAmplitudeB.train(dataAmplitudeB);
            dataAmplitudeB.clear();
        }
      }
      

      // Apply frequency adjustments to toggle pins at calculated intervals
      applyFrequencyAdjustment(coilAPin, softFilterFreqA.filter(thisFrequencyA), thisAmplitudeA, lastTimeA, stateA, currentTime, stateB);
      applyFrequencyAdjustment(coilBPin, softFilterFreqB.filter(thisFrequencyB), thisAmplitudeB, lastTimeB, stateB, currentTime, stateA);

      oscillate(lastTimeOscillation, currentTime);
      // Debug output
      Serial.print(" - Reading: ");
      Serial.print(currentA);
      Serial.print(" - Error: ");
      Serial.print(emfError);
      Serial.print(" - Amplitude A: ");
      Serial.print(thisAmplitudeA);
      Serial.print(" - Amplitude B: ");
      Serial.print(thisAmplitudeB);
      Serial.print(" - Frequency A: ");
      Serial.print(thisFrequencyA);
      Serial.print(" - Frequency B: ");
      Serial.println(thisFrequencyB);
    }



    // Frequency adjustments and calibration for coil A
    if (tuningFrequencyA && frequencyPIDA.calibrating()) {
        frequencyPIDA.calibrate(emfError, currentTime);
    }

    // Frequency adjustments and calibration for coil B
    if (!tuningFrequencyA && frequencyPIDB.calibrating()) {
        frequencyPIDB.calibrate(emfError, currentTime);
    }
    // Frequency adjustments and calibration for coil A
    if (tuningAmplitudeA && amplitudePIDA.calibrating()) {
        amplitudePIDA.calibrate(emfError, currentTime);
    }

    // Frequency adjustments and calibration for coil B
    if (!tuningAmplitudeA && amplitudePIDB.calibrating()) {
        amplitudePIDB.calibrate(emfError, currentTime);
    }


    tuningFrequencyA = !tuningFrequencyA;
    tuningAmplitudeA = !tuningAmplitudeA;

}


// Define a function to apply frequency adjustment using PID control
float applyFrequencyAdjustment(int pin, float frequency, float amplitude, long long &lastToggleTime, bool &pinState, long long currentTime, bool otherPinState) {
    // Convert frequency to interval in milliseconds; handle very high frequencies appropriately
    int toggleInterval = std::max(1, int(1000.0 / frequency)); // Calculate interval in ms, prevent zero division
    // Toggle the pin if enough time has elapsed and the other coil is not active
    if (currentTime - lastToggleTime >= toggleInterval) {
        pinState = !pinState;
        analogWrite(pin, amplitude); // Apply amplitude only when pin is HIGH
        lastToggleTime = currentTime;
    }

    return amplitude;
}


// Define a function to apply frequency adjustment using PID control
void oscillate(long long &lastTimeOscillation, long long currentTime) {
    if (currentTime - lastTimeOscillation <= 5) {
        myStepper.step(1);
    } else {

        myStepper.step(-1);

        // analogWrite(motorPins[0], LOW);
        // analogWrite(motorPins[1], LOW);
        // analogWrite(motorPins[2], LOW);
        // analogWrite(motorPins[3], LOW);

        if(currentTime - lastTimeOscillation >= 10){
          lastTimeOscillation = currentTime;
        }
    }
}
