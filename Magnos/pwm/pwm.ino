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
    int calibrationSteps = 64;

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

            relayState = !relayState;

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
        bool shouldToggle = error != prev_error;
    		if (shouldToggle) {
            relayState = !relayState;
        }
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

double oscillationFrequency = 4000000;
double blueLaserMultiplier = 660;
const double maxReading = 1023;

const int numTrainSamples = 32;
int predictionPhaseCounter = 0;

// PIDController oscillationPID(1.0, 0, 0.0); 

// LinearRegressionModel oscillationModel;

double interpolate(double current, double target, double step) {
    if (abs(current - target) < step) return target; // Close enough to target
    if (current < target) return current + step;     // Increment current
    if (current > target) return current - step;     // Decrement current
    return current;                                  // Default case (should not hit)
}

int oscillatorPins[3] = {7, 8, 9};

void setup() {
    // pinMode(receiverPinA, INPUT);
    // pinMode(A0, INPUT);
    Serial.begin(9600); // Set baud rate to 230400 bps

    // Start auto-tuning for frequency and amplitude PID controllers
    // oscillationPID.startAutoTuning(1, 0);

    pinMode(coilAPin, OUTPUT);
    pinMode(coilBPin, OUTPUT);


    analogWrite(coilAPin, 255);
    analogWrite(coilBPin, 255);

    pinMode(oscillatorPins[0], OUTPUT);
    pinMode(oscillatorPins[1], OUTPUT);
    pinMode(oscillatorPins[2], OUTPUT);
}

void loop() {

    // int sensorValue = analogRead(A0);  // Assuming the solar cell is connected to A0
    // float voltage = sensorValue * (5.0 / 1023.0);
    // Serial.print(" - Voltage ");
    // Serial.print(voltage);
    delayMicroseconds(1);

    // long long currentTime = micros();
    // static long long lastTime = currentTime;
    static int oscTurn = 0;
    // long long delta = (currentTime - lastTime); // Calculate delta time in milliseconds
    // double dt = (delta) / 1000.0; // Calculate delta time in seconds
    // lastTime = currentTime;

    // static int oscillationToggle = 0;

    // int currentA = analogRead(receiverPinA);
    // double emfError = (maxReading - currentA) / maxReading; // Adjusted error calculation


    if(oscTurn == 0){
      tone(oscillatorPins[0], oscillationFrequency / 2, 10);
      analogWrite(oscillatorPins[1], 0);
      analogWrite(oscillatorPins[2], 0);
      oscTurn = 1;
    } else if(oscTurn == 1){
      analogWrite(oscillatorPins[0], 0);
      tone(oscillatorPins[1], oscillationFrequency / 2, 10);
      analogWrite(oscillatorPins[2], 0);
      oscTurn = 2;
    } else {
      analogWrite(oscillatorPins[0], 0);
      analogWrite(oscillatorPins[1], 0);
      tone(oscillatorPins[2], blueLaserMultiplier, 1000);
      oscTurn = 0;
    }

    // bool calibrating = oscillationPID.calibrating();

    // if(calibrating){

    //   oscillationPID.calibrate(emfError, currentTime);

    //   oscillationToggle = oscillationPID.getRelayState();

    //   Serial.print(" - CAL: ");
    //   Serial.print(" - Reading: ");
    //   Serial.print(currentA);
    //   Serial.print(" - Error: ");
    //   Serial.println(emfError);

    // } else if(delta > 0){
    //   if(predictionPhaseCounter < numTrainSamples){
    //     int fakeComputing = oscillationPID.compute(emfError, dt);

    //     oscillationToggle = oscillationPID.getRelayState();

    //     predictionPhaseCounter++;

    //     Serial.print(" - PID: ");
        
    //   } else {

    //     predictionPhaseCounter++;
    //     if(predictionPhaseCounter > numTrainSamples * 2){
    //       predictionPhaseCounter = 0;
    //     }

    //     oscillationToggle = round(oscillationModel.predict(emfError));

    //     Serial.print(" - PRE: ");

    //   }

    //   if(predictionPhaseCounter < numTrainSamples){
    //     oscillationModel.addData({emfError, oscillationToggle});
    //   } 

    //   Serial.print(" - Reading: ");
    //   Serial.print(currentA);
    //   Serial.print(" - Error: ");
    //   Serial.println(emfError);
    // }



}
