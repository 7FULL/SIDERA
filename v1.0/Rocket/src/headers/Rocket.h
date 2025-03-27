#ifndef ROCKET_H
#define ROCKET_H

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "RF24.h"
#include "FS.h"
#include "OneWire.h"
#include "DallasTemperature.h"

enum BuzzerPattern {
    SILENT,
    SINGLE_BEEP,
    CONTINUOUS_BEEP,
    DOUBLE_BEEP,
    ERROR_BEEP,
    SOS,
    ASCENT_BEEP,
    DESCENT_BEEP,
    LANDED_BEEP
};

struct BuzzerState {
    BuzzerPattern pattern = SILENT;
    BuzzerPattern previousPattern = SILENT;
    unsigned long previousMillis = 0;
    bool buzzerState = false;
    int beepCounter = 0;
};

enum LedState {
    OFF,
    ON,
    BLINK,
    DOUBLE_BLINK
};

enum CommandCodes {
    CMD_PING = 1,
    CMD_DEPLOY_PARACHUTE = 2,
    CMD_ABORT = 3,
    CMD_REBOOT = 4,
    CMD_WAKE_UP = 5,
    CMD_LAUNCH = 6,
    CMD_START_PLATFORM = 7,
    CMD_ROCKET_READY = 8,
};

struct TelemetryData {
    float altitude;
    float temperature;
    float pressure;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float latitude;
    float longitude;
    bool parachuteDeployed;
    bool hasReachedApogee;
    bool landed;
    uint32_t timestamp;
};

struct CommandData {
    uint8_t command;
    uint32_t parameter;
};

struct Led {
    LedState state = OFF;
    LedState previousState = OFF;
    unsigned long previousMillis = 0;
    bool ledState = false;
    int blinkCounter = 0;
    uint8_t pin;
};

class Rocket {
public:
    Rocket();
    bool setup();
    bool checkSensors();
    void gravityIsOverrated();

    void updateBuzzer();

    BuzzerState buzzerState;

    Led blueLed;
    Led greenLed;
    Led redLed;

    void updateLeds();

    void resetLeds();

    bool isDescending();

    void deployParachute();

    void checkParachuteDescent();

    bool hasLanded();

    void landed();

    void logData(const String &message);

    void receiveCommands();

    bool hasReceivedWakeUp();
    bool hasReceivedLunch();

    bool sendRocketReady();

private:
    const byte readingAddress[6];
    const byte writingAddress[6];

    //VSPI - SD CARD
    const uint8_t vMisoPin;
    const uint8_t vMosiPin;
    const uint8_t vSckPin;
//    const uint8_t vCePin;
    const uint8_t vCsPin;

    //HSPI - RADIO
    const uint8_t hMisoPin;
    const uint8_t hMosiPin;
    const uint8_t hSckPin;
    const uint8_t hCePin;
    const uint8_t hCsPin;

    const uint8_t buzzerPin;

    //DS18B20
    const uint8_t ds18b20Pin;
    OneWire* oneWire;
    DallasTemperature* ds18b20;
    float ds18b20Temperature;

    const uint8_t rxPin;
    const uint8_t txPin;

    const uint8_t sdaPin;
    const uint8_t sclPin;

    const uint8_t blueLedPin;
    const uint8_t greenLedPin;
    const uint8_t redLedPin;

    const uint8_t parachutePin;

    float temperature, pressure, altitude, humidity, maxAltitude, groundAltitude;
    sensors_event_t accelerometer, gyroscope, tempEvent;
    bool gpsValid, hasReachedApogee, parachuteDeployed, receiveWakeUp, receiveLunch, hasRocketLanded;
    float gpsLatitude, gpsLongitude;
    String gpsDateTime;

    Adafruit_BME280 bme;
    Adafruit_MPU6050 mpu;
    TinyGPSPlus gps;
    SoftwareSerial ss;
    RF24 radio; // CE, CSN

    File logFile;
    String logFileName;

    bool initializeSensors();
    void readSensors();
    void printSensorData();

    void readGPS();
    void readBME280();
    void readMPU6050();

    void printGPSData();
    void printBME280Data();
    void printMPU6050Data();

    bool houstonWeHaveAProblem(const String &message);

    void setupLeds();

    void updateLed(Led &led);

    bool setupDoubleSpiBus();

    bool initializeLogging();

    bool initializeDS18B20();

    void readDS18B20();

    void logSensorData();

    TelemetryData telemetryData;
    CommandData commandData;

    // RF24 Communication functions
    bool initializeRF24();
    void sendTelemetry();
    bool processCommand(CommandData& command);

};

#endif