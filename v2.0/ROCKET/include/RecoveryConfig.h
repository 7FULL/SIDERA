#ifndef RECOVERY_CONFIG_H
#define RECOVERY_CONFIG_H

// Parachute parameters
#define PARACHUTE_DEPLOY_DURATION 1000   // ms
#define PARACHUTE_SERVO_DEPLOYED 180     // degrees
#define PARACHUTE_SERVO_STOWED 0         // degrees

// Deployment detection
#define DEPLOY_DETECTION_ACCEL_THRESHOLD 2.0f  // g
#define DEPLOY_CONFIRMATION_TIME 500     // ms

// Landing parameters
#define LANDING_VELOCITY_THRESHOLD 0.5f  // m/s
#define LANDING_STABILITY_TIME 5000      // ms

#endif