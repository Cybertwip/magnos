#include <Arduino.h>
// #include <ArduinoSTL.h>
// struct DataPoint {
//     double error;
//     double adjustment;
// };

// class LinearRegressionModel {
// private:
//     double intercept;
//     double slope;
//     double sum_x;
//     double sum_y;
//     double sum_xx;
//     double sum_xy;
//     int n;
//     bool trained;

// public:
    
//     LinearRegressionModel() : intercept(0), slope(0), sum_x(0), sum_y(0), sum_xx(0), sum_xy(0), n(0), trained(false) {}

//     void train(const std::vector<DataPoint>& data) {
//         for (const auto& point : data) {
//             addData(point);
//         }
//         trained = true;
//     }

//     void addData(const DataPoint& point) {
//         double old_mean_x = n > 0 ? sum_x / n : 0;
//         double old_mean_y = n > 0 ? sum_y / n : 0;

//         n++;
//         sum_x += point.error;
//         sum_y += point.adjustment;
        
//         double new_mean_x = sum_x / n;
//         double new_mean_y = sum_y / n;

//         sum_xx += (point.error - new_mean_x) * (point.error - old_mean_x);
//         sum_xy += (point.error - new_mean_x) * (point.adjustment - old_mean_y);

//         updateModel();
//     }

//     void updateModel() {
//         if (sum_xx != 0) {
//             slope = sum_xy / sum_xx;
//             intercept = sum_y / n - slope * sum_x / n;
//         } else {
//             slope = 0;  // Prevent division by zero
//             intercept = sum_y / n;  // All data points have the same x value
//         }
//     }


//     double predict(double desiredCurrent) {
//         return intercept + slope * desiredCurrent;
//     }

//     bool isTrained() const {
//       return trained;
//     }
// };



// class PIDController {
// private:
//     double kp, ki, kd;
//     double integral = 0.0;
//     double prev_error = 0.0;
//     bool autoTuning = false;
//     double relayHigh, relayLow;
//     long long prevSwitchTime = 0, oscillationStartTime = 0;
//     int oscillationsCount = 0;
//     double period = 0.0;
//     int calibrationSteps = 64;

//     bool relayState = true;

// public:
//     PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}

//     bool getRelayState() const { return relayState; }
//     void startAutoTuning(double relayHigh, double relayLow) {
//         if(!autoTuning){
//           autoTuning = true;
//         }
//         this->relayHigh = relayHigh;
//         this->relayLow = relayLow;
//         // Do not reset oscillationsCount and period here if resuming
//         if (oscillationsCount == 0) {
//             period = 0.0;
//             prevSwitchTime = micros(); // Re-initialize timing only if starting fresh
//         }
//     }

//     bool calibrating() { return autoTuning; }

//     void calibrate(double error, long long currentTime) {
//         if (!autoTuning) return;

//         bool shouldToggle = error != prev_error;
//         long long timeDelta = currentTime - prevSwitchTime;

//     		if (shouldToggle) {
//             if (oscillationsCount > 0) {
//                 period += timeDelta;
//             }
//             prevSwitchTime = currentTime;

//             relayState = !relayState;

//             oscillationsCount++;
//         }

//         prev_error = error;

//         if (oscillationsCount >= calibrationSteps) {
//             autoTuning = false;
//             period /= (oscillationsCount - 1);
//             kp = 0.6 * relayHigh / period;
//             ki = 2 * kp / period;
//             kd = kp * period / 2;

//             // Reset for possible future re-calibration
//             oscillationsCount = 0;

//         }
//     }

//     double compute(double error, double dt) {
//         bool shouldToggle = error != prev_error;
//     		if (shouldToggle) {
//             relayState = !relayState;
//         }
//         double p = kp * error;
//         integral += error * dt;
//         double d = kd * (error - prev_error) / dt;
//         double output = p + ki * integral + d;
//         prev_error = error;
//         return output;
//     }
// };

// class MovingAverageFilter {
// private:
//     std::vector<double> buffer;
//     size_t windowSize;
    
// public:
//     MovingAverageFilter(size_t size);
    
//     double filter(double inputValue);
// };

// MovingAverageFilter::MovingAverageFilter(size_t size) : windowSize(size) {}

// double MovingAverageFilter::filter(double inputValue) {
//     buffer.push_back(inputValue);
//     if (buffer.size() > windowSize) {
//         buffer.erase(buffer.begin());
//     }
    
//     double sum = 0.0f;
//     for (const double& value : buffer) {
//         sum += value;
//     }
//     return sum / buffer.size();
// }


// MovingAverageFilter softFilter(255);

// Define constants and create PIDController instances
const int coilPinA = 2;
const int coilPinB = 3;
const int coilPinC = 7;
// int oscillatorPinA = 9;
// int oscillatorPinB = 10;

// const int oscPinA = 6;
// const int oscPinB = 7;
// const int oscPinC = 8;
// const int oscPinD = 9;


// const int receiverPinA = A4;

// double oscillationFrequency = 4000000;
// double blueLaserMultiplierA = 10000;
// double blueLaserMultiplierB = 3000;
// const double maxReading = 1023;

// const int numTrainSamples = 32;
// int predictionPhaseCounter = 0;

// double interpolate(double current, double target, double step) {
//     if (abs(current - target) < step) return target; // Close enough to target
//     if (current < target) return current + step;     // Increment current
//     if (current > target) return current - step;     // Decrement current
//     return current;                                  // Default case (should not hit)
// }


void setup() {

    pinMode(coilPinA, OUTPUT);
    pinMode(coilPinB, OUTPUT);
    pinMode(coilPinC, OUTPUT);

    analogWrite(coilPinA, 254);
    analogWrite(coilPinB, 254);
    analogWrite(coilPinC, 254);

}

void loop() {
}
