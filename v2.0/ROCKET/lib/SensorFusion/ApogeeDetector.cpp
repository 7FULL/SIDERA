/**
 * Apogee Detector Implementation
 */

#include "ApogeeDetector.h"

ApogeeDetector::ApogeeDetector(BarometricSensorManager* baroManager, IMUSensorManager* imuManager, KalmanFilter* altitudeFilter)
        : baroManager(baroManager),
          imuManager(imuManager),
          altitudeFilter(altitudeFilter),
          detectionThreshold(2.0f),    // 2 meters below max altitude
          confirmationSamples(5),      // 5 consecutive readings confirming descent
          minimumAltitude(10.0f),      // Minimum 10 meters to avoid ground detection
          apogeeDetected(false),
          apogeeAltitude(0.0f),
          maxAltitude(0.0f),
          apogeeTime(0),
          descentCount(0),
          lastUpdateTime(0),
          confidence(0.0f) {
}

bool ApogeeDetector::begin() {
    if (!baroManager || !imuManager || !altitudeFilter) {
        return false;
    }

    // Initialize data structures
    altitudeHistory.clear();
    velocityHistory.clear();

    apogeeDetected = false;
    apogeeAltitude = 0.0f;
    maxAltitude = 0.0f;
    apogeeTime = 0;
    descentCount = 0;
    confidence = 0.0f;

    lastUpdateTime = millis();

    setMinimumAltitude(50.0f);

    return true;
}

void ApogeeDetector::update() {
    if (!baroManager || !imuManager || !altitudeFilter) {
        return;
    }

    // Get current altitude from Kalman filter (which should be more reliable)
    float currentAltitude = altitudeFilter->getPosition();
    float currentVelocity = altitudeFilter->getVelocity();

    // Update data history
    updateAltitudeHistory(currentAltitude);
    updateVelocityHistory(currentVelocity);

    // Update max altitude
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
    }

    // Check if we already detected apogee
    if (apogeeDetected) {
        return;
    }

    // Skip detection if we haven't reached minimum altitude
    if (maxAltitude < minimumAltitude) {
        return;
    }

    // Run apogee detection algorithms
    bool basicDetection = detectApogeeBasic();
    bool advancedDetection = detectApogeeAdvanced();

    // Confirm apogee if both methods agree or one is very confident
    if ((basicDetection && advancedDetection) ||
        (advancedDetection && confidence > 0.8f)) {

        apogeeDetected = true;
        apogeeAltitude = maxAltitude;
        apogeeTime = millis();
    }
}

void ApogeeDetector::reset() {
    altitudeHistory.clear();
    velocityHistory.clear();

    apogeeDetected = false;
    apogeeAltitude = 0.0f;
    maxAltitude = 0.0f;
    apogeeTime = 0;
    descentCount = 0;
    confidence = 0.0f;

    lastUpdateTime = millis();
}

float ApogeeDetector::getConfidence() const {
    return confidence;
}

bool ApogeeDetector::isApogeeDetected() const {
    return apogeeDetected;
}

float ApogeeDetector::getApogeeAltitude() const {
    return apogeeAltitude;
}

float ApogeeDetector::getMaxAltitude() const {
    return maxAltitude;
}

unsigned long ApogeeDetector::getApogeeTime() const {
    return apogeeTime;
}

void ApogeeDetector::setDetectionThreshold(float threshold) {
    detectionThreshold = threshold;
}

void ApogeeDetector::setConfirmationSamples(int samples) {
    confirmationSamples = samples > 0 ? samples : 1;
}

void ApogeeDetector::setMinimumAltitude(float minAltitude) {
    minimumAltitude = minAltitude;
}

bool ApogeeDetector::detectApogeeBasic() {
    // Basic apogee detection based on altitude decrease
    // We need at least a few samples to work with
    if (altitudeHistory.size() < confirmationSamples) {
        return false;
    }

    // Check if current altitude is significantly below max altitude
    float currentAltitude = altitudeHistory.back();
    if (maxAltitude - currentAltitude > detectionThreshold) {
        descentCount++;
    } else {
        descentCount = 0; // Reset count if not descending
    }

    // Apogee confirmed if we have enough consecutive descent samples
    return (descentCount >= confirmationSamples);
}

bool ApogeeDetector::detectApogeeAdvanced() {
    // Advanced apogee detection using multiple criteria

    // Need enough samples for velocity calculation
    if (velocityHistory.size() < confirmationSamples) {
        return false;
    }

    // Get current velocity (from Kalman filter)
    float currentVelocity = velocityHistory.back();

    // Calculate average velocity over the last few samples
    float sumVelocity = 0.0f;
    for (const auto& velocity : velocityHistory) {
        sumVelocity += velocity;
    }
    float avgVelocity = sumVelocity / velocityHistory.size();

    // Check if velocity has consistently been negative (descending)
    bool velocityCheck = (currentVelocity < -0.5f) && (avgVelocity < -0.2f);

    // Check altitude derivative for inflection point
    float altitudeDerivative = computeAltitudeDerivative();
    bool derivativeCheck = (altitudeDerivative < 0.0f);

    // Check if acceleration is near gravity (free fall)
    AccelerometerData accelData = imuManager->getAccelerometerData();
    bool accelerationCheck = (accelData.magnitude < 10.5f) && (accelData.magnitude > 9.0f);

    // Combine checks with weights
    float velocityWeight = 0.5f;
    float derivativeWeight = 0.3f;
    float accelerationWeight = 0.2f;

    float combinedConfidence =
            (velocityCheck ? velocityWeight : 0.0f) +
            (derivativeCheck ? derivativeWeight : 0.0f) +
            (accelerationCheck ? accelerationWeight : 0.0f);

    // Update confidence (with smoothing)
    confidence = lowPassFilter(combinedConfidence, confidence, 0.3f);

    // Apogee is detected if combined confidence is high enough
    return (confidence > 0.6f);
}

float ApogeeDetector::computeAltitudeDerivative() {
    // Compute numerical derivative of altitude (second derivative - acceleration)
    // Need at least 3 samples for second derivative
    if (altitudeHistory.size() < 3) {
        return 0.0f;
    }

    // Get the last 3 altitude samples
    float altitude0 = altitudeHistory[altitudeHistory.size() - 3];
    float altitude1 = altitudeHistory[altitudeHistory.size() - 2];
    float altitude2 = altitudeHistory[altitudeHistory.size() - 1];

    // Compute first derivatives
    float derivative1 = altitude1 - altitude0;
    float derivative2 = altitude2 - altitude1;

    // Compute second derivative
    float secondDerivative = derivative2 - derivative1;

    return secondDerivative;
}

void ApogeeDetector::updateAltitudeHistory(float altitude) {
    // Add new altitude to history
    altitudeHistory.push_back(altitude);

    // Keep history size limited
    const size_t MAX_HISTORY = 20;
    while (altitudeHistory.size() > MAX_HISTORY) {
        altitudeHistory.pop_front();
    }
}

void ApogeeDetector::updateVelocityHistory(float velocity) {
    // Add new velocity to history
    velocityHistory.push_back(velocity);

    // Keep history size limited
    const size_t MAX_HISTORY = 10;
    while (velocityHistory.size() > MAX_HISTORY) {
        velocityHistory.pop_front();
    }
}