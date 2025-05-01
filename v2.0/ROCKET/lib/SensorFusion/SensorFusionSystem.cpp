/**
 * Sensor Fusion System Implementation
 */

#include "SensorFusionSystem.h"

SensorFusionSystem::SensorFusionSystem(
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        StorageManager* storageManager
)
        : baroManager(baroManager),
          imuManager(imuManager),
          storageManager(storageManager),
          orientationFilter(nullptr),
          altitudeFilter(nullptr),
          apogeeDetector(nullptr),
          lastUpdateTime(0),
          lastLogTime(0),
          lastVerticalSpeed(0.0f),
          verticalAcceleration(0.0f) {

    // Initialize fused data
    fusedData = {0};
    fusedData.confidence = 0.0f;
}

SensorFusionSystem::~SensorFusionSystem() {
    // Clean up allocated filters
    if (orientationFilter) delete orientationFilter;
    if (altitudeFilter) delete altitudeFilter;
    if (apogeeDetector) delete apogeeDetector;
}

bool SensorFusionSystem::begin() {
    // Check required managers
    if (!baroManager || !imuManager) {
        return false;
    }

    // Create fusion algorithms
    orientationFilter = new ComplementaryFilter(imuManager);
    altitudeFilter = new KalmanFilter();

    // Configure altitude filter
    // These values are tuned for a typical rocket flight
    altitudeFilter->setProcessNoise(0.01f, 0.1f); // Low position noise, higher velocity noise
    altitudeFilter->setPredictionFactor(1.0f);    // Standard prediction model

    // Initialize with current altitude
    float initialAltitude = baroManager->getAltitude();
    altitudeFilter->setState(initialAltitude, 0.0f);

    // Create apogee detector
    apogeeDetector = new ApogeeDetector(baroManager, imuManager, altitudeFilter);

    // Configure apogee detector
    apogeeDetector->setDetectionThreshold(1.5f); // 1.5 meters threshold
    apogeeDetector->setConfirmationSamples(3);   // 3 samples to confirm
    apogeeDetector->setMinimumAltitude(10.0f);   // Minimum 10 meters

    // Initialize all fusion algorithms
    bool initSuccess = true;
    initSuccess &= orientationFilter->begin();
    initSuccess &= altitudeFilter->begin();
    initSuccess &= apogeeDetector->begin();

    if (!initSuccess) {
        if (storageManager) {
            storageManager->logMessage(
                    LogLevel::ERROR,
                    Subsystem::SENSORS,
                    "Failed to initialize sensor fusion system"
            );
        }
        return false;
    }

    // Initialize timing
    lastUpdateTime = millis();
    lastLogTime = lastUpdateTime;

    if (storageManager) {
        storageManager->logMessage(
                LogLevel::INFO,
                Subsystem::SENSORS,
                "Sensor fusion system initialized successfully"
        );
    }

    return true;
}

void SensorFusionSystem::update() {
    // Calculate time delta
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;  // in seconds
    lastUpdateTime = currentTime;

    // Sanity check on dt
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;  // Default to 10ms if time delta is invalid
    }

    // Update sensor fusion algorithms
    updateAltitudeEstimation();
    updateOrientationEstimation();
    detectFlightEvents();
    calculateConfidence();

    // Log data periodically
    if (currentTime - lastLogTime >= 500) {  // Log every 500ms
        lastLogTime = currentTime;
        logFusionData();
    }

    // Calculate vertical acceleration
    float currentSpeed = altitudeFilter->getVelocity();
    verticalAcceleration = (currentSpeed - lastVerticalSpeed) / dt;
    lastVerticalSpeed = currentSpeed;

    // Update fused data structure
    fusedData.altitude = altitudeFilter->getPosition();
    fusedData.verticalSpeed = altitudeFilter->getVelocity();
    fusedData.verticalAccel = verticalAcceleration;

    fusedData.roll = orientationFilter->getRoll();
    fusedData.pitch = orientationFilter->getPitch();
    fusedData.yaw = orientationFilter->getYaw();

    fusedData.apogeeDetected = apogeeDetector->isApogeeDetected();
    fusedData.apogeeAltitude = apogeeDetector->getApogeeAltitude();
    fusedData.maxAltitude = apogeeDetector->getMaxAltitude();
}

void SensorFusionSystem::reset() {
    // Reset all fusion algorithms
    if (orientationFilter) orientationFilter->reset();
    if (altitudeFilter) altitudeFilter->reset();
    if (apogeeDetector) apogeeDetector->reset();

    // Reset fused data
    fusedData = {0};
    fusedData.confidence = 0.0f;

    // Reset timing
    lastUpdateTime = millis();
    lastLogTime = lastUpdateTime;
    lastVerticalSpeed = 0.0f;
    verticalAcceleration = 0.0f;

    if (storageManager) {
        storageManager->logMessage(
                LogLevel::INFO,
                Subsystem::SENSORS,
                "Sensor fusion system reset"
        );
    }
}

FusedFlightData SensorFusionSystem::getFusedData() const {
    return fusedData;
}

float SensorFusionSystem::getAltitude() const {
    return fusedData.altitude;
}

float SensorFusionSystem::getVerticalSpeed() const {
    return fusedData.verticalSpeed;
}

float SensorFusionSystem::getVerticalAcceleration() const {
    return fusedData.verticalAccel;
}

float SensorFusionSystem::getRoll() const {
    return fusedData.roll;
}

float SensorFusionSystem::getPitch() const {
    return fusedData.pitch;
}

float SensorFusionSystem::getYaw() const {
    return fusedData.yaw;
}

bool SensorFusionSystem::isApogeeDetected() const {
    return fusedData.apogeeDetected;
}

float SensorFusionSystem::getApogeeAltitude() const {
    return fusedData.apogeeAltitude;
}

float SensorFusionSystem::getMaxAltitude() const {
    return fusedData.maxAltitude;
}

float SensorFusionSystem::getAltitudeConfidence() const {
    return altitudeFilter ? altitudeFilter->getConfidence() : 0.0f;
}

float SensorFusionSystem::getOrientationConfidence() const {
    return orientationFilter ? orientationFilter->getConfidence() : 0.0f;
}

float SensorFusionSystem::getOverallConfidence() const {
    return fusedData.confidence;
}

void SensorFusionSystem::setCurrentState(RocketState state) {
    currentState = state;
}

void SensorFusionSystem::updateAltitudeEstimation() {
    if (!baroManager || !altitudeFilter) {
        return;
    }

    // Get raw altitude from barometric sensor
    float rawAltitude = baroManager->getAltitude();

    // Update Kalman filter with barometric measurement
    // Higher uncertainty during high acceleration phases
    float altitudeUncertainty = 1.0f;  // Base uncertainty

    if (imuManager) {
        AccelerometerData accelData = imuManager->getAccelerometerData();

        // Increase uncertainty when under high acceleration
        float accelMagnitudeError = abs(accelData.magnitude - 9.81f);
        altitudeUncertainty += accelMagnitudeError * 0.2f;
    }

    // Update filter with new measurement
    altitudeFilter->updateWithPosition(rawAltitude, altitudeUncertainty);

    // Predict next state
    altitudeFilter->update();
}

void SensorFusionSystem::updateOrientationEstimation() {
    if (!imuManager || !orientationFilter) {
        return;
    }

    // Update orientation filter
    orientationFilter->update();
}

void SensorFusionSystem::detectFlightEvents() {
    if (!apogeeDetector) {
        return;
    }

    // Update apogee detector
    if (currentState == RocketState::COASTING ||
        currentState == RocketState::POWERED_FLIGHT) {
        apogeeDetector->update();
    }
}

void SensorFusionSystem::calculateConfidence() {
    if (!altitudeFilter || !orientationFilter) {
        fusedData.confidence = 0.0f;
        return;
    }

    // Calculate overall confidence based on individual confidences
    float altitudeConfidence = altitudeFilter->getConfidence();
    float orientationConfidence = orientationFilter->getConfidence();
    float apogeeConfidence = apogeeDetector ? apogeeDetector->getConfidence() : 0.0f;

    Serial.printf("Altitude Confidence: %.2f, Orientation Confidence: %.2f, Apogee Confidence: %.2f\n",
           altitudeConfidence, orientationConfidence, apogeeConfidence);

    // Weight the confidences based on importance
    const float ALTITUDE_WEIGHT = 0.5f;
    const float ORIENTATION_WEIGHT = 0.3f;
    const float APOGEE_WEIGHT = 0.2f;

    float weightedConfidence =
            altitudeConfidence * ALTITUDE_WEIGHT +
            orientationConfidence * ORIENTATION_WEIGHT +
            apogeeConfidence * APOGEE_WEIGHT;

    // Apply a low-pass filter for smoother confidence changes
    fusedData.confidence = 0.9f * fusedData.confidence + 0.1f * weightedConfidence;
}

void SensorFusionSystem::logFusionData() {
    if (!storageManager) {
        return;
    }

    // Log important fusion data periodically
    char message[64];

    // Log altitude and vertical speed
    snprintf(
            message, sizeof(message),
            "Alt: %.1fm, VSpeed: %.1fm/s, Conf: %.2f",
            fusedData.altitude,
            fusedData.verticalSpeed,
            fusedData.confidence
    );
    storageManager->logMessage(LogLevel::DEBUG, Subsystem::SENSORS, message);

    // Log apogee detection if it happened
    if (fusedData.apogeeDetected) {
        snprintf(
                message, sizeof(message),
                "APOGEE DETECTED at %.1fm",
                fusedData.apogeeAltitude
        );
        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, message);
    }
}