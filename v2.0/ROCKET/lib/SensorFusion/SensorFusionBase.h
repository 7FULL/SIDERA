/**
 * Sensor Fusion Base
 *
 * Abstract base class for sensor fusion algorithms
 */

#ifndef SENSOR_FUSION_BASE_H
#define SENSOR_FUSION_BASE_H

#include <Arduino.h>

class SensorFusionBase {
public:
    virtual ~SensorFusionBase() = default;

    // Initialize the fusion algorithm
    virtual bool begin() = 0;

    // Update the fusion algorithm with new sensor data
    virtual void update() = 0;

    // Reset the fusion algorithm to initial state
    virtual void reset() = 0;

    // Get estimated confidence in current values (0.0 - 1.0)
    virtual float getConfidence() const = 0;

protected:
    // Common utility functions

    // Simple low-pass filter
    float lowPassFilter(float newValue, float prevValue, float alpha) {
        return alpha * newValue + (1.0f - alpha) * prevValue;
    }

    // Deadband filter (ignore small changes)
    float deadbandFilter(float newValue, float prevValue, float threshold) {
        if (abs(newValue - prevValue) < threshold) {
            return prevValue;
        }
        return newValue;
    }

    // Map a value from one range to another
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Constrain a value to a range
    float constrainFloat(float x, float min_val, float max_val) {
        if (x < min_val) return min_val;
        if (x > max_val) return max_val;
        return x;
    }
};

#endif // SENSOR_FUSION_BASE_H