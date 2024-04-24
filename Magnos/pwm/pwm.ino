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
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;
    int n;
    bool trained;

public:
    
    LinearRegressionModel() : intercept(0), slope(0), sum_x(0), sum_y(0), sum_xx(0), sum_xy(0), n(0), trained(false) {}



    void train(const std::vector<DataPoint>& data) {
        for (const auto& point : data) {
            addData(point);
        }
        updateModel();

        trained = true;
    }

    void addData(const DataPoint& point) {
        double old_mean_x = n > 0 ? sum_x / n : 0;
        double old_mean_y = n > 0 ? sum_y / n : 0;

        n++;
        sum_x += point.error;
        sum_y += point.finalCurrent;
        
        double new_mean_x = sum_x / n;
        double new_mean_y = sum_y / n;

        sum_xx += (point.error - new_mean_x) * (point.error - old_mean_x);
        sum_xy += (point.error - new_mean_x) * (point.finalCurrent - old_mean_y);

        updateModel();
    }

    void updateModel() {
        if (sum_xx != 0) {
            slope = sum_xy / sum_xx;
            intercept = sum_y / n - slope * sum_x / n;
        } else {
            slope = 0;  // Prevent division by zero
            intercept = sum_y / n;  // All data points have the same x value
        }
    }


    double predict(double desiredCurrent) {
        return intercept + slope * desiredCurrent;
    }

    bool isTrained() const {
      return trained;
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
    int calibrationSteps = 8;

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
const int maxFrequencyA = 32000;
const int maxFrequencyB = 64000;
const double maxCurrentA = 255;
const double maxCurrentB = 200;
const double maxReading = 1023;

const int numTrainSamples = 128;
int predictionPhaseCounter = 0;

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

const double targetReading = maxReading; // Example: 75% of maximum reading

LinearRegressionModel mlModel;

std::vector<DataPoint> data;

double clamp(double value, double min, double max) {
    // If value is less than min, return min
    if (value < min) return min;

    // If value is greater than max, return max
    if (value > max) return max;

    // Otherwise, return the value itself
    return value;
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
    frequencyPIDA.startAutoTuning(1, 0);
    frequencyPIDB.startAutoTuning(1, 0);
    amplitudePIDA.startAutoTuning(1, 0);
    amplitudePIDB.startAutoTuning(1, 0);

    lastTimeA = millis();
    lastTimeB = millis();
    lastTimeOscillation = millis();

    myStepper.setSpeed(100);
    pinMode(motorPins[0], OUTPUT);
    pinMode(motorPins[1], OUTPUT);

    analogWrite(coilAPin, maxCurrentA);
    analogWrite(coilBPin, maxCurrentB);
}

void loop() {
    long long currentTime = millis();
    static long long lastTime = currentTime;
    long long delta = (currentTime - lastTime); // Calculate delta time in milliseconds
    double dt = (delta) / 1000.0; // Calculate delta time in seconds
    lastTime = currentTime;

    int currentA = analogRead(receiverPinA);
    double emfError = (targetReading - currentA) / targetReading; // Adjusted error calculation


    static double lowestEmfError = 1.0;
    
    if(currentA >= targetReading){
      return;
    } 
    

    frequencyPIDA.startAutoTuning(1, 0);
    frequencyPIDB.startAutoTuning(1, 0);
    amplitudePIDA.startAutoTuning(1, 0);
    amplitudePIDB.startAutoTuning(1, 0);

    // Compute PID outputs for frequency and amplitude control for coils A and B
    if(delta > 0){
      double frequencyAdjustmentA = 0;
      double frequencyAdjustmentB = 0;
      double amplitudeAdjustmentA = 0;
      double amplitudeAdjustmentB = 0;
      if(predictionPhaseCounter < numTrainSamples){
        frequencyAdjustmentA = frequencyPIDA.compute((emfError * maxFrequencyA) / maxFrequencyA, dt) * maxFrequencyA;
        frequencyAdjustmentB = frequencyPIDB.compute((emfError * maxFrequencyB) / maxFrequencyB, dt) * maxFrequencyB;
        amplitudeAdjustmentA = amplitudePIDA.compute((emfError * maxCurrentA) / maxCurrentA, dt) * maxCurrentA;
        amplitudeAdjustmentB = amplitudePIDB.compute((emfError * maxCurrentB) / maxCurrentB, dt) * maxCurrentB;
        // Clamp the adjusted amplitude within the valid range
        thisAmplitudeA = clamp(amplitudeAdjustmentA, 0.0, maxCurrentA);
        thisAmplitudeB = clamp(amplitudeAdjustmentB, 0.0, maxCurrentB);

        thisFrequencyA = clamp(frequencyAdjustmentA, 0.0, maxFrequencyA);
        thisFrequencyB = clamp(frequencyAdjustmentB, 0.0, maxFrequencyB);


        predictionPhaseCounter++;

        Serial.print(" - P: ");
        Serial.print(" - Reading: ");
        Serial.print(currentA);
        Serial.print(" - Error: ");
        Serial.println(emfError);
        
      } else {

        predictionPhaseCounter++;
        if(predictionPhaseCounter > numTrainSamples * 2){
          predictionPhaseCounter = 0;
        }

        
        double prediction = mlModel.predict(emfError);

        prediction = std::max(0.0, prediction);  // Use max to ensure non-negative without losing negative info

        if (targetReading > 0) {  // Ensure targetReading is positive to avoid division by zero
            double scale_factor = prediction / targetReading;

            double frequencyAdjustmentA = scale_factor * maxFrequencyA;
            double frequencyAdjustmentB = scale_factor * maxFrequencyB;
            double amplitudeAdjustmentA = scale_factor * maxCurrentA;
            double amplitudeAdjustmentB = scale_factor * maxCurrentB;

            // Clamp the adjusted values within the valid range
            thisAmplitudeA = clamp(amplitudeAdjustmentA, 0.0, maxCurrentA);
            thisAmplitudeB = clamp(amplitudeAdjustmentB, 0.0, maxCurrentB);
            thisFrequencyA = clamp(frequencyAdjustmentA, 0.0, maxFrequencyA);
            thisFrequencyB = clamp(frequencyAdjustmentB, 0.0, maxFrequencyB);
            
            Serial.print(" - T: ");
            Serial.print(" - Reading: ");
            Serial.print(currentA);
            Serial.print(" - Error: ");
            Serial.println(emfError);     
        } else {
            Serial.println("Invalid targetReading value; must be > 0");
        }


      }

      if(data.size() < numTrainSamples){
        DataPoint trainData;
        
        trainData.error = emfError;
        trainData.finalCurrent = currentA;

        data.push_back(trainData);

      } else {
        // The buffer is full, replace the oldest data point
        // Since std::vector does not support pop_front, we use a different approach
        std::rotate(data.begin(), data.begin() + 1, data.end()); // Shift left by one
        data.back().error = emfError; // Update the last element with new data
        data.back().finalCurrent = currentA;
      }
      
      if(emfError < lowestEmfError){

        if (data.size() >= numTrainSamples) {
            mlModel.train(data);
            data.clear(); // Clear after training
        }

        lowestEmfError = emfError;
      }

      // Apply frequency adjustments to toggle pins at calculated intervals
      applyFrequencyAdjustment(coilAPin, thisFrequencyA, thisAmplitudeA, lastTimeA, stateA, currentTime, stateB);
      applyFrequencyAdjustment(coilBPin, thisFrequencyB, thisAmplitudeB, lastTimeB, stateB, currentTime, stateA);

      // oscillate(lastTimeOscillation, currentTime);
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
