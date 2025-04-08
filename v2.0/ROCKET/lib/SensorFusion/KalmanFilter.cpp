/**
 * Kalman Filter Implementation
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
        : predictionFactor(1.0f),
          lastUpdateTime(0),
          confidence(0.0f) {

    // Initialize state vector
    stateVector[0] = 0.0f;  // Initial position
    stateVector[1] = 0.0f;  // Initial velocity

    // Initialize covariance matrix with high uncertainty
    covarianceMatrix[0][0] = 100.0f; // Position variance
    covarianceMatrix[0][1] = 0.0f;   // Position-velocity covariance
    covarianceMatrix[1][0] = 0.0f;   // Velocity-position covariance
    covarianceMatrix[1][1] = 100.0f; // Velocity variance

    // Default process noise
    processNoise[0] = 0.01f; // Position process noise
    processNoise[1] = 0.1f;  // Velocity process noise
}

bool KalmanFilter::begin() {
    lastUpdateTime = millis();
    confidence = 0.5f; // Start with moderate confidence
    return true;
}

void KalmanFilter::update() {
    // Calculate time delta
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;  // in seconds
    lastUpdateTime = currentTime;

    // Sanity check on dt
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;  // Default to 10ms if time delta is invalid
    }

    // Predict next state
    predict(dt);
}

void KalmanFilter::reset() {
    // Reset state vector
    stateVector[0] = 0.0f;
    stateVector[1] = 0.0f;

    // Reset covariance matrix with high uncertainty
    covarianceMatrix[0][0] = 100.0f;
    covarianceMatrix[0][1] = 0.0f;
    covarianceMatrix[1][0] = 0.0f;
    covarianceMatrix[1][1] = 100.0f;

    lastUpdateTime = millis();
    confidence = 0.0f;
}

float KalmanFilter::getConfidence() const {
    return confidence;
}

void KalmanFilter::setState(float position, float velocity) {
    stateVector[0] = position;
    stateVector[1] = velocity;
}

float KalmanFilter::getPosition() const {
    return stateVector[0];
}

float KalmanFilter::getVelocity() const {
    return stateVector[1];
}

void KalmanFilter::updateWithPosition(float measurement, float uncertainty) {
    updateMeasurement(measurement, uncertainty, 0);
}

void KalmanFilter::updateWithAcceleration(float measurement, float uncertainty, float dt) {
    // Acceleration affects velocity, which in turn affects position
    // First, update the state prediction with the acceleration
    float acceleration = measurement;

    // Update velocity: v = v + a*dt
    stateVector[1] += acceleration * dt;

    // Update position: p = p + v*dt + 0.5*a*dt^2
    stateVector[0] += stateVector[1] * dt + 0.5f * acceleration * dt * dt;

    // Increase uncertainty (covariance) due to acceleration
    covarianceMatrix[1][1] += uncertainty * dt * dt;
    covarianceMatrix[0][0] += covarianceMatrix[1][1] * dt * dt;
    covarianceMatrix[0][1] += covarianceMatrix[1][1] * dt;
    covarianceMatrix[1][0] += covarianceMatrix[1][1] * dt;
}

void KalmanFilter::setProcessNoise(float positionNoise, float velocityNoise) {
    processNoise[0] = positionNoise;
    processNoise[1] = velocityNoise;
}

void KalmanFilter::setPredictionFactor(float factor) {
    predictionFactor = factor;
}

void KalmanFilter::predict(float dt) {
    // State prediction:
    // position = position + velocity * dt
    // velocity = velocity

    // Apply non-linear adjustment factor for better predictions in a rocket context
    float adjustedDt = dt * predictionFactor;

    // Update state vector
    stateVector[0] += stateVector[1] * adjustedDt;

    // Update covariance matrix
    // P = F * P * F' + Q
    // Where F is the state transition matrix and Q is the process noise covariance

    // F = [1 dt; 0 1]
    // F' = [1 0; dt 1]

    float temp00 = covarianceMatrix[0][0] + dt * (covarianceMatrix[1][0] + covarianceMatrix[0][1] + dt * covarianceMatrix[1][1]);
    float temp01 = covarianceMatrix[0][1] + dt * covarianceMatrix[1][1];
    float temp10 = covarianceMatrix[1][0] + dt * covarianceMatrix[1][1];
    float temp11 = covarianceMatrix[1][1];

    covarianceMatrix[0][0] = temp00 + processNoise[0];
    covarianceMatrix[0][1] = temp01;
    covarianceMatrix[1][0] = temp10;
    covarianceMatrix[1][1] = temp11 + processNoise[1];
}

void KalmanFilter::updateMeasurement(float measurement, float uncertainty, int measurementIndex) {
    // Kalman gain calculation
    // K = P * H' * inv(H * P * H' + R)
    // Where H is the measurement matrix and R is the measurement noise covariance

    // For position measurement (index 0), H = [1 0]
    // For velocity measurement (index 1), H = [0 1]

    float k0, k1; // Kalman gains for state vector elements

    if (measurementIndex == 0) {
        // Position measurement
        float denominator = covarianceMatrix[0][0] + uncertainty;
        if (denominator < 1e-6f) denominator = 1e-6f; // Avoid division by zero

        k0 = covarianceMatrix[0][0] / denominator;
        k1 = covarianceMatrix[1][0] / denominator;

        // Update state vector
        float innovation = measurement - stateVector[0];
        stateVector[0] += k0 * innovation;
        stateVector[1] += k1 * innovation;

        // Update covariance matrix
        covarianceMatrix[0][0] = (1.0f - k0) * covarianceMatrix[0][0];
        covarianceMatrix[0][1] = (1.0f - k0) * covarianceMatrix[0][1];
        covarianceMatrix[1][0] = -k1 * covarianceMatrix[0][0] + covarianceMatrix[1][0];
        covarianceMatrix[1][1] = -k1 * covarianceMatrix[0][1] + covarianceMatrix[1][1];
    } else {
        // Velocity measurement
        float denominator = covarianceMatrix[1][1] + uncertainty;
        if (denominator < 1e-6f) denominator = 1e-6f; // Avoid division by zero

        k0 = covarianceMatrix[0][1] / denominator;
        k1 = covarianceMatrix[1][1] / denominator;

        // Update state vector
        float innovation = measurement - stateVector[1];
        stateVector[0] += k0 * innovation;
        stateVector[1] += k1 * innovation;

        // Update covariance matrix
        covarianceMatrix[0][0] = -k0 * covarianceMatrix[1][0] + covarianceMatrix[0][0];
        covarianceMatrix[0][1] = -k0 * covarianceMatrix[1][1] + covarianceMatrix[0][1];
        covarianceMatrix[1][0] = (1.0f - k1) * covarianceMatrix[1][0];
        covarianceMatrix[1][1] = (1.0f - k1) * covarianceMatrix[1][1];
    }

    // Update confidence based on covariance
    // Lower covariance = higher confidence
    float positionVariance = covarianceMatrix[0][0];
    float velocityVariance = covarianceMatrix[1][1];

    // Confidence is inversely proportional to variance
    float positionConfidence = 1.0f / (1.0f + positionVariance);
    float velocityConfidence = 1.0f / (1.0f + velocityVariance);

    // Overall confidence is average of position and velocity confidence
    confidence = (positionConfidence + velocityConfidence) / 2.0f;
}