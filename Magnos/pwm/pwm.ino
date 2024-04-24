#include <Arduino.h>
#include <ArduinoSTL.h>

#include <Stepper.h>

struct DataPoint {
    double error;
    double finalCurrent;
};

class LinearRegressionModel {
private:
    double intercept;
    double slope;

public:
    LinearRegressionModel() : intercept(0), slope(0) {}

    void train(const std::vector<DataPoint>& data) {
        int n = data.size();
        if (n == 0) return;

        double sum_x = 0, sum_y = 0, mean_x = 0, mean_y = 0;
        for (const auto& point : data) {
            sum_x += point.error;
            sum_y += point.finalCurrent;
        }
        mean_x = sum_x / n;
        mean_y = sum_y / n;

        double sum_xx = 0, sum_xy = 0;
        for (const auto& point : data) {
            double x_i = point.error - mean_x;
            double y_i = point.finalCurrent - mean_y;
            sum_xx += x_i * x_i;
            sum_xy += x_i * y_i;
        }

        double denominator = sum_xx;
        if (denominator != 0) {
            slope = sum_xy / denominator;
            intercept = mean_y - slope * mean_x;
        } else {
            slope = 0;  // Prevent division by zero
            intercept = mean_y; // Or consider other fallbacks
        }
    }

    double predict(double desiredCurrent) {
        return intercept + slope * desiredCurrent;
    }
};


class PIDController {
private:
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;
    bool autoTuning = false;
    double relayHigh, relayLow;
    long long prevSwitchTime = 0, oscillationStartTime = 0;
    int oscillationsCount = 0;
    double period = 0.0;
    int calibrationSteps = 32;

public:
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}

    void startAutoTuning(double relayHigh, double relayLow) {
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

    void calibrate(double error, long long currentTime) {
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
            kp = 0.6 * relayHigh / period;
            ki = 2 * kp / period;
            kd = kp * period / 8;

            // Reset for possible future re-calibration
            oscillationsCount = 0;

        }
    }

    double compute(double error, double dt) {
        double p = kp * error;
        integral += error * dt;
        double d = kd * (error - prev_error) / dt;
        double output = p + ki * integral + d;
        prev_error = error;
        return output;
    }
};

class MovingAverageFilter {
private:
    std::vector<double> buffer;
    size_t windowSize;
    
public:
    MovingAverageFilter(size_t size);
    
    double filter(double inputValue);
};

MovingAverageFilter::MovingAverageFilter(size_t size) : windowSize(size) {}

double MovingAverageFilter::filter(double inputValue) {
    buffer.push_back(inputValue);
    if (buffer.size() > windowSize) {
        buffer.erase(buffer.begin());
    }
    
    double sum = 0.0f;
    for (const double& value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}


// Define constants and create PIDController instances
const int coilAPin = 2;
const int coilBPin = 3;
const int receiverPinA = A4;
const int maxFrequencyA = 12000;
const int maxFrequencyB = 12000;
const double maxCurrent = 255;
const double maxReading = 1023;

const int numTrainSamples = 10;

PIDController frequencyPIDA(1.0, 0, 0); 
PIDController frequencyPIDB(1.0, 0, 0); 
PIDController amplitudePIDA(1.0, 0, 0); 
PIDController amplitudePIDB(1.0, 0, 0); 


bool tuningFrequencyA = true;
bool tuningAmplitudeA = true;

long long lastTimeA = 0;
long long lastTimeB = 0;

long long lastTimeOscillation = 0;

bool stateA = false;
bool stateB = false;

double thisFrequencyA = 0;
double thisFrequencyB = 0;
double thisAmplitudeA = 0;
double thisAmplitudeB = 0;

const double targetCurrent = maxReading * 0.85; // Example: 75% of maximum reading

LinearRegressionModel mlFrequencyA;
LinearRegressionModel mlFrequencyB;
LinearRegressionModel mlAmplitudeA;
LinearRegressionModel mlAmplitudeB;

std::vector<DataPoint> dataFrequencyA, dataFrequencyB;
std::vector<DataPoint> dataAmplitudeA, dataAmplitudeB;

MovingAverageFilter softFilter(64);

// Define the clamp function
double clamp(double value, double min, double max) {
    return value < min ? min : (value > max ? max : value);
}

double interpolate(double current, double target, double step) {
    if (abs(current - target) < step) return target; // Close enough to target
    if (current < target) return current + step;     // Increment current
    if (current > target) return current - step;     // Decrement current
    return current;                                  // Default case (should not hit)
}


// Number of steps per revolution of the internal gear shaft
int stepsPerRevolution = 4096; // Change this depending on your motor's specification

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
    double dt = (delta) / 1000.0; // Calculate delta time in seconds
    lastTime = currentTime;

    int currentA = softFilter.filter(analogRead(receiverPinA));

    if(currentA >= 900){
      return;
    }
    
    double emfError = ((targetCurrent - currentA) / maxReading); // Adjusted error calculation
    double desiredCurrent = (1.0 - emfError) * maxCurrent;

    frequencyPIDA.startAutoTuning(maxFrequencyA, 0);
    frequencyPIDB.startAutoTuning(maxFrequencyB, 0);
    amplitudePIDA.startAutoTuning(maxReading, 0);
    amplitudePIDB.startAutoTuning(maxReading, 0);

    // Compute PID outputs for frequency and amplitude control for coils A and B
    if(delta > 0){

      double frequencyAdjustmentA;
      double frequencyAdjustmentB;
      double amplitudeAdjustmentA;
      double amplitudeAdjustmentB;
      if(dataFrequencyA.size() < numTrainSamples){
        frequencyAdjustmentA = frequencyPIDA.compute(emfError, dt) * maxFrequencyA;
        frequencyAdjustmentB = frequencyPIDB.compute(emfError, dt) * maxFrequencyB;
        amplitudeAdjustmentA = amplitudePIDA.compute(emfError, dt) * maxCurrent;
        amplitudeAdjustmentB = amplitudePIDB.compute(emfError, dt) * maxCurrent;
      } else {
        frequencyAdjustmentA = mlFrequencyA.predict(emfError);
        frequencyAdjustmentB = mlFrequencyB.predict(emfError);
        amplitudeAdjustmentA = mlAmplitudeA.predict(emfError);
        amplitudeAdjustmentB = mlAmplitudeB.predict(emfError);
      }

      // Smoothly adjust amplitude by adding the adjustment to the current amplitude
      thisAmplitudeA = interpolate(thisAmplitudeA, amplitudeAdjustmentA, 1);
      thisAmplitudeB = interpolate(thisAmplitudeB, amplitudeAdjustmentB, 1);

      // Clamp the adjusted amplitude within the valid range
      thisAmplitudeA = clamp(amplitudeAdjustmentA, 0.0f, maxCurrent);
      thisAmplitudeB = clamp(amplitudeAdjustmentB, 0.0f, maxCurrent);

      // Map the amplitude values to the range [0, 255]
      thisAmplitudeA = map(thisAmplitudeA, 0, maxCurrent, 0, 255);
      thisAmplitudeB = map(thisAmplitudeB, 0, maxCurrent, 0, 255);

      thisFrequencyA = interpolate(thisFrequencyA, frequencyAdjustmentA, 1);
      thisFrequencyB = interpolate(thisFrequencyB, frequencyAdjustmentB, 1);


      thisFrequencyA = clamp(frequencyAdjustmentA, 0.0f, maxFrequencyA);
      thisFrequencyB = clamp(frequencyAdjustmentB, 0.0f, maxFrequencyB);


      if(dataFrequencyA.size() < numTrainSamples){
        DataPoint frequencyDataA;
        DataPoint frequencyDataB;
        DataPoint amplitudeDataA;
        DataPoint amplitudeDataB;

        frequencyDataA.error = emfError;
        frequencyDataB.error = emfError;
        amplitudeDataA.error = desiredCurrent;
        amplitudeDataB.error = desiredCurrent;

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

            mlAmplitudeA.train(dataAmplitudeA);
            dataAmplitudeA.clear();

            mlFrequencyB.train(dataFrequencyB);
            dataFrequencyB.clear();

            mlAmplitudeB.train(dataAmplitudeB);
            dataAmplitudeB.clear();
        }

      }
      

      // Apply frequency adjustments to toggle pins at calculated intervals
      applyFrequencyAdjustment(coilAPin, thisFrequencyA, thisAmplitudeA, lastTimeA, stateA, currentTime, stateB);
      applyFrequencyAdjustment(coilBPin, thisFrequencyB, thisAmplitudeB, lastTimeB, stateB, currentTime, stateA);

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
double applyFrequencyAdjustment(int pin, double frequency, double amplitude, long long &lastToggleTime, bool &pinState, long long currentTime, bool otherPinState) {
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
    if (currentTime - lastTimeOscillation <= 3) {
        myStepper.step(1);
    } else {

        myStepper.step(-1);

        // analogWrite(motorPins[0], LOW);
        // analogWrite(motorPins[1], LOW);
        // analogWrite(motorPins[2], LOW);
        // analogWrite(motorPins[3], LOW);

        if(currentTime - lastTimeOscillation >= 7){
          lastTimeOscillation = currentTime;
        }
    }
}
