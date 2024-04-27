#include <Arduino.h>
#include <ArduinoSTL.h>

struct DataPoint {
    double error;
    double adjustment;
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
        trained = true;
    }

    void addData(const DataPoint& point) {
        double old_mean_x = n > 0 ? sum_x / n : 0;
        double old_mean_y = n > 0 ? sum_y / n : 0;

        n++;
        sum_x += point.error;
        sum_y += point.adjustment;
        
        double new_mean_x = sum_x / n;
        double new_mean_y = sum_y / n;

        sum_xx += (point.error - new_mean_x) * (point.error - old_mean_x);
        sum_xy += (point.error - new_mean_x) * (point.adjustment - old_mean_y);

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
    int calibrationSteps = 4096;

    bool relayState = true;

public:
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}

    bool getRelayState() const { return relayState; }
    void startAutoTuning(double relayHigh, double relayLow) {
        if(!autoTuning){
          autoTuning = true;
        }
        this->relayHigh = relayHigh;
        this->relayLow = relayLow;
        // Do not reset oscillationsCount and period here if resuming
        if (oscillationsCount == 0) {
            period = 0.0;
            prevSwitchTime = micros(); // Re-initialize timing only if starting fresh
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

            relayState = ~relayState;

            oscillationsCount++;
        }

        prev_error = error;

        if (oscillationsCount >= calibrationSteps) {
            autoTuning = false;
            period /= (oscillationsCount - 1);
            kp = 0.6 * relayHigh / period;
            ki = 2 * kp / period;
            kd = kp * period / 2;

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


MovingAverageFilter softFilter(255);

// Define constants and create PIDController instances
const int coilAPin = 2;
const int coilBPin = 3;
const int receiverPinA = A4;


const int magnosPinIn = A7;
const int magnosPinOut = 12;

double maxFrequencyA = 4000;
double maxFrequencyB = 4300;
double maxCurrentA = 255;
double maxCurrentB = 255;
const double maxReading = 1023;
long long accumulationTime = 0;

const int numTrainSamples = 256;
int predictionPhaseCounter = 0;

PIDController frequencyPID(1.0, 0, 0.0); 
PIDController amplitudePID(1.0, 0, 0.0); 

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

LinearRegressionModel frequencyModel;
LinearRegressionModel amplitudeModel;

double interpolate(double current, double target, double step) {
    if (abs(current - target) < step) return target; // Close enough to target
    if (current < target) return current + step;     // Increment current
    if (current > target) return current - step;     // Decrement current
    return current;                                  // Default case (should not hit)
}

// Pins connected from the oscillator to the driver IN1, IN2
int oscillatorPins[4] = {8, 9}; // Only using 2 pins for simplicity
bool oscillationToggle = true;

void setup() {

    pinMode(coilAPin, OUTPUT);
    pinMode(coilBPin, OUTPUT);

    pinMode(magnosPinIn, INPUT);
    pinMode(magnosPinOut, OUTPUT);

    pinMode(receiverPinA, INPUT);
    Serial.begin(230400); // Set baud rate to 230400 bps

    // Start auto-tuning for frequency and amplitude PID controllers
    frequencyPID.startAutoTuning(maxFrequencyA, 0);
    amplitudePID.startAutoTuning(maxCurrentA, 0);

    lastTimeA = micros();
    lastTimeB = micros();
    lastTimeOscillation = micros();
    pinMode(oscillatorPins[0], OUTPUT);
    pinMode(oscillatorPins[1], OUTPUT);

    analogWrite(coilAPin, maxCurrentA);
    analogWrite(coilBPin, maxCurrentB);
}

void loop() {
    long long currentTime = micros();
    static long long lastTime = currentTime;
    long long delta = (currentTime - lastTime); // Calculate delta time in milliseconds
    double dt = (delta) / 1000.0; // Calculate delta time in seconds
    lastTime = currentTime;

    int currentA = analogRead(receiverPinA);
    double emfError = (targetReading - currentA) / targetReading; // Adjusted error calculation
    
    
    int magnosInput = analogRead(magnosPinIn);

    if(magnosInput > 128){
      analogWrite(magnosPinOut, 0);


      if(oscillationToggle){
        analogWrite(oscillatorPins[1], 255);
      } else {
        analogWrite(oscillatorPins[1], 0);
      }

    } else {

      analogWrite(magnosPinOut, 255);
    }
    bool calibrating = frequencyPID.calibrating() || amplitudePID.calibrating();

    if(currentA >= targetReading && !calibrating){

      oscillate(lastTimeOscillation, currentTime, 1000,  1000);

      return;
    }


    if(calibrating){
      accumulationTime += delta;

      frequencyPID.calibrate(emfError, currentTime);
      amplitudePID.calibrate(emfError, currentTime);

      thisFrequencyA = frequencyPID.getRelayState() ? maxFrequencyA : 0;
      thisFrequencyB = thisFrequencyA * sin(M_PI / 4); 

      thisAmplitudeA = amplitudePID.getRelayState() ? maxCurrentA : 0;
      thisAmplitudeB = thisAmplitudeA * cos(M_PI / 4); 

      applyFrequencyAdjustment(coilAPin, thisFrequencyA, thisAmplitudeA, lastTimeA, stateA, currentTime, stateB);
      applyFrequencyAdjustment(coilBPin, thisFrequencyB, thisAmplitudeA, lastTimeB, stateB, currentTime, stateA);

      oscillationToggle = amplitudePID.getRelayState() ? true : false;

      Serial.print(" - CAL: ");

    } else if(delta > 0){
      if(predictionPhaseCounter < numTrainSamples){
        double frequencyAdjustmentA = frequencyPID.compute(emfError, dt) * maxFrequencyA;
        double thetaFrequencyB = thisFrequencyA * sin(M_PI / 4); 

        double amplitudeAdjustmentA = amplitudePID.compute(emfError, dt) * maxCurrentA;
        double amplitudeAdjustmentB = amplitudeAdjustmentA * cos(M_PI / 4); 

        thisFrequencyA = std::max(0.0, std::min(frequencyAdjustmentA, maxFrequencyA));
        thisFrequencyB = std::max(0.0, std::min(thetaFrequencyB, maxFrequencyB));
        thisAmplitudeA = std::max(0.0, std::min(amplitudeAdjustmentA, maxCurrentA));
        thisAmplitudeB = std::max(0.0, std::min(amplitudeAdjustmentB, maxCurrentB));

        predictionPhaseCounter++;


            Serial.print(" - PID: ");

        
      } else {

        predictionPhaseCounter++;
        if(predictionPhaseCounter > numTrainSamples * 2){
          predictionPhaseCounter = 0;
        }

        if (targetReading > 0) {  // Ensure targetReading is positive to avoid division by zero

            double frequencyAdjustmentA = frequencyModel.predict(emfError);
            double frequencyAdjustmentB = frequencyAdjustmentA * sin(M_PI / 4); 
            double amplitudeAdjustmentA = amplitudeModel.predict(emfError);
            double amplitudeAdjustmentB = amplitudeAdjustmentA * cos(M_PI / 4); 

            // Clamp the adjusted values within the valid range

            thisFrequencyA = std::max(0.0, std::min(frequencyAdjustmentA, maxFrequencyA));
            thisFrequencyB = std::max(0.0, std::min(frequencyAdjustmentB, maxFrequencyB));
            thisAmplitudeA = std::max(0.0, std::min(amplitudeAdjustmentA, maxCurrentA));
            thisAmplitudeB = std::max(0.0, std::min(amplitudeAdjustmentB, maxCurrentB));

            
            Serial.print(" - PRE: ");

        } else {
            Serial.println("Invalid targetReading value; must be > 0");
        }


      }

      if(predictionPhaseCounter < numTrainSamples){
        frequencyModel.addData({emfError, thisFrequencyA});
        amplitudeModel.addData({emfError, thisAmplitudeA});
      } 

      // Apply frequency adjustments to toggle pins at calculated intervals


      applyFrequencyAdjustment(coilAPin, thisFrequencyA, thisAmplitudeA, lastTimeA, stateA, currentTime, stateB);
      applyFrequencyAdjustment(coilBPin, thisFrequencyB, thisAmplitudeB, lastTimeB, stateB, currentTime, stateA);

      oscillate(lastTimeOscillation, currentTime, 1,  1);
    }


      Serial.print(" - Reading: ");
      Serial.print(currentA);
      Serial.print(" - Short: ");
      Serial.print(magnosInput);
      Serial.print(" - Error: ");
      Serial.print(emfError);

      Serial.print(" - FA: ");
      Serial.print(thisFrequencyA);
      Serial.print(" - FB: ");
      Serial.print(thisFrequencyB);

      Serial.print(" - AA: ");
      Serial.print(thisAmplitudeA);
      Serial.print(" - AB: ");
      Serial.println(thisAmplitudeB);

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


void oscillate(long long &lastTimeOscillation, long long currentTime, double frequencyOn, double frequencyOff) {
    // Convert frequency in Hz to interval in milliseconds
    int onInterval = (frequencyOn > 0) ? static_cast<int>(1000 / frequencyOn) : INT8_MAX;  // Prevent division by zero
    int offInterval = (frequencyOff > 0) ? static_cast<int>(1000 / frequencyOff) : INT8_MAX;  // Prevent division by zero

    // Toggle logic
    if (oscillationToggle && (currentTime - lastTimeOscillation >= onInterval)) {
        oscillationToggle = false;  // Turn off
        lastTimeOscillation = currentTime;  // Update the last toggle time
    } else if (!oscillationToggle && (currentTime - lastTimeOscillation >= offInterval)) {
        oscillationToggle = true;   // Turn on
        lastTimeOscillation = currentTime;  // Update the last toggle time
    }
}

