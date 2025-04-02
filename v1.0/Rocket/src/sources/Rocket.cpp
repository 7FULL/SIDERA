#include "headers/Rocket.h"
#include <SD.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// We use 9600 baud because is the NEO-8M if it's a NEO-6M use 4800 baud
#define GPSBaud 9600

SPIClass vspi(VSPI);
SPIClass hspi(HSPI);

Rocket::Rocket()
    :   readingAddress("00002")
    ,   writingAddress("00001")
    ,   buzzerPin(27)
    ,   rxPin(16)
    ,   txPin(17)
    ,   sdaPin(21)
    ,   sclPin(22)
    ,   blueLedPin(33)
    ,   greenLedPin(26)//NO USADO EN v2.0 ya que se enciende automatico al alimentar
    ,   redLedPin(25)
    ,   parachutePin(26)
    ,   vMisoPin(19)
    ,   vMosiPin(23)
    ,   vSckPin(18)
    ,   vCsPin(5)
    ,   hMisoPin(12)
    ,   hMosiPin(13)
    ,   hSckPin(14)
    ,   hCePin(4)
    ,   hCsPin(15)
    ,   ds18b20Pin(0)
    ,   radio(hCePin, hCsPin)
{
    blueLed.state = BLINK;
    greenLed.state = BLINK;
    redLed.state = BLINK;

    hasReachedApogee = false;
    parachuteDeployed = false;
    receiveWakeUp = false;
}

bool Rocket::setup() {
    setupLeds();

    ledcSetup(0, 1000, 8);
    ledcAttachPin(buzzerPin, 0);
    ledcWriteTone(0, 0);

    pinMode(parachutePin, OUTPUT);
    pinMode(ds18b20Pin, INPUT);

    if (!setupDoubleSpiBus()){
        return houstonWeHaveAProblem("[ERROR] Error initializing SPI buses");
    }

    if (!initializeLogging()){
        return houstonWeHaveAProblem("[ERROR] Error initializing logging");
    }

    if (!initializeSensors()) {
        Serial.println("Failed to initialize sensors!");
        return houstonWeHaveAProblem("[ERROR] Error initializing sensors");
    }

    resetLeds();

    logData("[INFO] Rocket initialized");

    buzzerState.pattern = SINGLE_BEEP;
    greenLed.state = ON;

    hasRocketLanded = false;

    return true;
}

bool Rocket::initializeDS18B20() {
    oneWire = new OneWire(ds18b20Pin);
    ds18b20 = new DallasTemperature(oneWire);

    ds18b20->begin();

    int deviceCount = ds18b20->getDeviceCount();
    if (deviceCount == 0) {
        houstonWeHaveAProblem("[ERROR] No DS18B20 sensors found");
        return false;
    }

    ds18b20->setResolution(12);

    return true;
}

void Rocket::readDS18B20() {
    ds18b20->requestTemperatures();
    ds18b20Temperature = ds18b20->getTempCByIndex(0);

    if (ds18b20Temperature == DEVICE_DISCONNECTED_C) {
        houstonWeHaveAProblem("[ERROR] Error reading DS18B20");
        ds18b20Temperature = -127.0;
    }
}

bool Rocket::setupDoubleSpiBus() {
    logData("[INFO] Setting up SPI buses...");

    pinMode(vSckPin, OUTPUT);
    pinMode(vMisoPin, INPUT);
    pinMode(vMosiPin, OUTPUT);
    pinMode(vCsPin, OUTPUT);

    // Configurar VSPI (SD CARD)
    vspi.begin(vSckPin, vMisoPin, vMosiPin, vCsPin);

    if (!SD.begin(vCsPin)) {
        Serial.println("SD Card Initialization Failed!");

        // Check common SD card initialization errors
        uint8_t cardType = SD.cardType();
        switch(cardType) {
            case CARD_NONE:
                Serial.println("No SD card attached");
                logData("[ERROR] No SD card detected");
                break;
            case CARD_MMC:
                Serial.println("MMC card detected but failed to initialize");
                break;
            case CARD_SD:
                Serial.println("SD card detected but failed to initialize");
                break;
            case CARD_SDHC:
                Serial.println("SDHC card detected but failed to initialize");
                break;
            default:
                Serial.println("Unknown card type");
        }

        // Advanced debugging
        Serial.print("VSPI SCK Pin: "); Serial.println(vSckPin);
        Serial.print("VSPI MISO Pin: "); Serial.println(vMisoPin);
        Serial.print("VSPI MOSI Pin: "); Serial.println(vMosiPin);
        Serial.print("VSPI CS Pin: "); Serial.println(vCsPin);

        return houstonWeHaveAProblem("[ERROR] Error initializing SD card");
    }

    Serial.println("SD Card initialized successfully");
    logData("[INFO] SD Card initialized");

    hspi.begin(hSckPin, hMisoPin, hMosiPin, hCsPin);

    if (!initializeRF24()){
        return houstonWeHaveAProblem("[ERROR] Error initializing RF24");
    }

    return true;
}

bool Rocket::initializeSensors() {
    logData("[INFO] Initializing sensors...");

    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return houstonWeHaveAProblem("[ERROR] Could not find a valid BME280 sensor, check wiring!");
    }

    bme.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X16,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::FILTER_OFF,
            Adafruit_BME280::STANDBY_MS_0_5
    );

    Wire.begin(sdaPin, sclPin);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        return houstonWeHaveAProblem("[ERROR] Failed to find MPU6050 chip");
    }

    if (!initializeDS18B20()){
        return houstonWeHaveAProblem("[ERROR] Error initializing DS18B20");
    }

    ss.begin(GPSBaud, SWSERIAL_8N1, rxPin, txPin);
    return true;
}

bool Rocket::checkSensors() {
    logData("[INFO] Checking sensors...");

    logData("[INFO] Reading DS18B20 sensor...");

    readDS18B20();

    if (ds18b20Temperature == -127.0) {
        return houstonWeHaveAProblem("[ERROR] Failed to read DS18B20 sensor");
    }

    logData("[INFO] DS18B20 sensor OK");
    logSensorData();

    logData("[INFO] Reading BME280 sensor...");

    readBME280();

    if (isnan(temperature) || isnan(pressure) || isnan(altitude) || isnan(humidity)) {
        return houstonWeHaveAProblem("[ERROR] Failed to read BME280 sensor");
    }

//    printBME280Data();

    logData("[INFO] BME280 sensor OK");
    logSensorData();

    logData("[INFO] Reading MPU6050 sensor...");

    readMPU6050();

    if (isnan(accelerometer.acceleration.x) || isnan(accelerometer.acceleration.y) || isnan(accelerometer.acceleration.z) ||
        isnan(gyroscope.gyro.x) || isnan(gyroscope.gyro.y) || isnan(gyroscope.gyro.z)) {

        return houstonWeHaveAProblem("[ERROR] Failed to read MPU6050 sensor");
    }

//    printMPU6050Data();

    logData("[INFO] MPU6050 sensor OK");
    logSensorData();

    logData("[INFO] Reading GPS sensor...");

    readGPS();

    if (gpsLatitude == 0.0 || gpsLongitude == 0.0 || gpsDateTime == "") {
        // printGPSData();

        return houstonWeHaveAProblem("[ERROR] Failed to read GPS sensor");
    }

//    printGPSData();

    blueLed.state = DOUBLE_BLINK;
    buzzerState.pattern = DOUBLE_BEEP;

    groundAltitude = altitude;

    logData("[INFO] Ground altitude: " + String(groundAltitude) + "m");

    logData("[INFO] GPS sensor OK");
    logSensorData();

    logData("[INFO] Sensors OK");

    logSensorData();

    return true;
}

void Rocket::readSensors() {
    readBME280();
    readMPU6050();
    readGPS();
    readDS18B20();

    logSensorData();
}

void Rocket::readBME280() {
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
}

void Rocket::readMPU6050() {
    mpu.getEvent(&accelerometer, &gyroscope, &tempEvent);
    gyroscope.gyro.x *= (180 / PI);
    gyroscope.gyro.y *= (180 / PI);
    gyroscope.gyro.z *= (180 / PI);
}

void Rocket::readGPS() {
    gpsValid = false;
    unsigned long start = millis();
    //5 minutes
    const unsigned long timeout = 300000;

    while (millis() - start < timeout) {
        // Have to update the buzzer and leds in the loop to avoid blocking the program
        updateLeds();
        updateBuzzer();

        delay(10);

        while (ss.available() > 0) {
            if (gps.encode(ss.read())) {
                if (gps.location.isValid()) {
                    gpsLatitude = gps.location.lat();
                    gpsLongitude = gps.location.lng();
                    gpsValid = true;
                }
                if (gps.date.isValid() && gps.time.isValid()) {
                    gpsDateTime = String(gps.date.month()) + "/" +
                                  String(gps.date.day()) + "/" +
                                  String(gps.date.year()) + " " +
                                  String(gps.time.hour()) + ":" +
                                  String(gps.time.minute()) + ":" +
                                  String(gps.time.second());
                }
            }
        }
        if (gpsValid) {
            break;
        }
    }
}

void Rocket::printSensorData() {
    printBME280Data();
    printMPU6050Data();
    printGPSData();
}

void Rocket::printBME280Data() {
    Serial.print("Temperature = "); Serial.print(temperature); Serial.println("*C");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println("hPa");
    Serial.print("Approx. Altitude = "); Serial.print(altitude); Serial.println("m");
    Serial.print("Humidity = "); Serial.print(humidity); Serial.println("%");
}

void Rocket::printMPU6050Data() {
    Serial.print("Aceleración X: "); Serial.print(accelerometer.acceleration.x); Serial.println(" m/s^2");
    Serial.print("Aceleración Y: "); Serial.print(accelerometer.acceleration.y); Serial.println(" m/s^2");
    Serial.print("Aceleración Z: "); Serial.print(accelerometer.acceleration.z); Serial.println(" m/s^2");
    Serial.print("Giroscopio X: "); Serial.print(gyroscope.gyro.x); Serial.println(" deg/s");
    Serial.print("Giroscopio Y: "); Serial.print(gyroscope.gyro.y); Serial.println(" deg/s");
    Serial.print("Giroscopio Z: "); Serial.print(gyroscope.gyro.z); Serial.println(" deg/s");
}

void Rocket::printGPSData() {
    Serial.print("Latitude: "); Serial.println(gpsLatitude, 6);
    Serial.print("Longitude: "); Serial.println(gpsLongitude, 6);
    Serial.print("Date/Time: "); Serial.println(gpsDateTime);

    if (!gpsValid) {
        Serial.println("GPS data is invalid!");
    }
}

void Rocket::gravityIsOverrated() {
    blueLed.state = OFF;
    greenLed.state = BLINK;
    buzzerState.pattern = ASCENT_BEEP;

    readSensors();

    sendTelemetry();

    receiveCommands();

    // printSensorData();
    // delay(5000);
}

bool Rocket::isDescending() {
    sendTelemetry();

    receiveCommands();

    // Reduced reading interval - every 100ms instead of 1000ms
    static const unsigned long CHECK_INTERVAL = 100;

    // Larger buffer for high temporal resolution
    static const int BUFFER_SIZE = 20;
    static float altitudeBuffer[BUFFER_SIZE] = {0};
    static int bufferIndex = 0;
    static bool bufferFilled = false;

    // Tracking variables
    static unsigned long lastCheckTime = 0;
    static float highestAltitude = 0;
    static int descentCounter = 0;
    static int consecutiveDescents = 0;
    static const int REQUIRED_DESCENTS = 5; // More samples to confirm descent

    // Adjustable threshold based on altitude and vertical speed
    static const float BASE_DESCENT_THRESHOLD = 0.2; // More sensitive (in meters)
    static float descentThreshold = BASE_DESCENT_THRESHOLD;

    // Only update at the specified frequency
    if (millis() - lastCheckTime >= CHECK_INTERVAL) {
        lastCheckTime = millis();

        // Update the maximum altitude
        if (altitude > highestAltitude) {
            highestAltitude = altitude;
            maxAltitude = highestAltitude; // Update global variable
        }

        // Add new reading to the circular buffer
        altitudeBuffer[bufferIndex] = altitude;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        // Mark when the buffer has filled for the first time
        if (bufferIndex == 0 && !bufferFilled) {
            bufferFilled = true;
        }

        // Only analyze when we have enough data
        if (bufferFilled || bufferIndex >= 10) {
            // Calculate short-term trend (last 5 readings)
            float shortTermAvg = 0;
            for (int i = 0; i < 5; i++) {
                int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
                shortTermAvg += altitudeBuffer[idx];
            }
            shortTermAvg /= 5;

            // Calculate medium-term trend (last 10 readings)
            float mediumTermAvg = 0;
            for (int i = 0; i < 10; i++) {
                int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
                mediumTermAvg += altitudeBuffer[idx];
            }
            mediumTermAvg /= 10;

            // Calculate rate of change (approximate vertical speed in m/s)
            float verticalRate = (altitudeBuffer[(bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE] -
                                  altitudeBuffer[(bufferIndex - 5 + BUFFER_SIZE) % BUFFER_SIZE]) /
                                 (5 * CHECK_INTERVAL / 1000.0);

            // Adjust detection threshold based on vertical speed
            // More stringent at higher altitude, more permissive at lower altitude
            descentThreshold = BASE_DESCENT_THRESHOLD + abs(verticalRate) * 0.1;

            // Apogee detection (more precise, based on trend change)
            if (!hasReachedApogee && highestAltitude > 10) { // Ignore the first few meters
                // Check if the trend changed from rising to falling
                if (verticalRate < -0.5 && altitude < highestAltitude - 0.5) {
                    hasReachedApogee = true;
                    Serial.println("APOGEE DETECTED!");
                    Serial.print("Max altitude: "); Serial.println(highestAltitude);
                    Serial.print("Vertical rate: "); Serial.print(verticalRate); Serial.println(" m/s");

                    // Log if you have that functionality
                    logData("[INFO] Apogee detected at " + String(highestAltitude) + " m");
                    return false; // Do not immediately activate descent, just notify apogee
                }
            }

            // Verify descent only after apogee
            if (hasReachedApogee) {
                // Multi-criteria strategy to detect descent:
                bool isCurrentlyDescending = false;

                // 1. Detect by difference between recent readings and average
                if (altitude < shortTermAvg - descentThreshold) {
                    isCurrentlyDescending = true;
                }

                // 2. Confirm by negative rate of change
                if (verticalRate < -1.0) { // Minimum descent speed (1 m/s)
                    isCurrentlyDescending = true;
                }

                // 3. Confirm by difference with maximum altitude
                if (altitude < highestAltitude - 5.0) { // At least 5m below the maximum
                    isCurrentlyDescending = true;
                }

                if (isCurrentlyDescending) {
                    consecutiveDescents++;

                    // Log for debugging (every second)
                    if (consecutiveDescents % 10 == 0) {
                        Serial.print("Potential descent "); Serial.print(consecutiveDescents);
                        Serial.print("/"); Serial.print(REQUIRED_DESCENTS);
                        Serial.print(" Alt: "); Serial.print(altitude);
                        Serial.print(" Rate: "); Serial.print(verticalRate);
                        Serial.println(" m/s");
                    }

                    if (consecutiveDescents >= REQUIRED_DESCENTS) {
                        Serial.println("DESCENT CONFIRMED!");
                        Serial.print("Current altitude: "); Serial.print(altitude);
                        Serial.print(" | Short term avg: "); Serial.println(shortTermAvg);
                        Serial.print("Vertical rate: "); Serial.print(verticalRate); Serial.println(" m/s");
                        Serial.print("Highest altitude reached: "); Serial.println(highestAltitude);

                        // Log if you have that functionality
                        logData("[INFO] Descent confirmed at " + String(altitude) + " m");

                        blueLed.state = DOUBLE_BLINK;
                        buzzerState.pattern = DESCENT_BEEP;

                        return true;
                    }
                } else {
                    // Gradually reduce the counter, do not reset it completely
                    // to avoid false interruptions during descent
                    if (consecutiveDescents > 0) {
                        consecutiveDescents--;
                    }
                }
            }
        }
    }

    return false;
}

void Rocket::deployParachute() {
    parachuteDeployed = true;
    digitalWrite(parachutePin, HIGH);

    logData("[INFO] Parachute deployed!");
}

void Rocket::checkParachuteDescent() {
    readSensors();

    static unsigned long parachuteDeployTime = 0;
    const unsigned long DEPLOY_STABILIZATION_TIME = 500;
    static float descentRateHistory[3] = {0};
    static int descentRateIndex = 0;
    static float avgDescentRate = 0;

    if (!parachuteDeployed) {
        deployParachute();
        parachuteDeployTime = millis();
        Serial.println("PARACHUTE DEPLOYED!");
        logData("FLIGHT: Parachute deployed at " + String(altitude) + "m");
    }

    if (millis() - parachuteDeployTime > DEPLOY_STABILIZATION_TIME) {
        static float lastAltitude = altitude;
        static unsigned long lastAltitudeTime = millis();

        if (millis() - lastAltitudeTime >= 50) {
            float timeDelta = (millis() - lastAltitudeTime) / 1000.0;

            if (timeDelta > 0.01) {
                float instantDescentRate = (lastAltitude - altitude) / timeDelta;

                descentRateHistory[descentRateIndex] = instantDescentRate;
                descentRateIndex = (descentRateIndex + 1) % 3;

                avgDescentRate = 0;
                for (int i = 0; i < 3; i++) {
                    avgDescentRate += descentRateHistory[i];
                }
                avgDescentRate /= 3;

                lastAltitude = altitude;
                lastAltitudeTime = millis();

                if (millis() % 500 < 50) {
                    logData("FLIGHT: Descent rate: " + String(avgDescentRate) + " m/s at altitude " + String(altitude) + "m");
                    Serial.print("Descent rate: "); Serial.print(avgDescentRate); Serial.println(" m/s");
                }

                const float MAX_SAFE_DESCENT_RATE = 8.0; // m/s
                const float MIN_EXPECTED_DESCENT_RATE = 0.5; // m/s

                if (avgDescentRate > MAX_SAFE_DESCENT_RATE) {
                    Serial.println("WARNING: Descent rate too high! Parachute may have failed!");
                    redLed.state = BLINK;
                    buzzerState.pattern = SOS;
                    houstonWeHaveAProblem("[ALERT] Descent rate too high (" + String(avgDescentRate) + "m/s)! Parachute failure suspected!");
                }
                else if (altitude < maxAltitude - 20 && avgDescentRate < MIN_EXPECTED_DESCENT_RATE) {
                    Serial.println("WARNING: Descent rate too low! Parachute may be stuck!");
                    redLed.state = DOUBLE_BLINK;
                    houstonWeHaveAProblem("[ALERT] Descent rate too low (" + String(avgDescentRate) + "m/s)! Parachute stuck suspected!");
                }
            }
        }
    }
}

bool Rocket::hasLanded() {
    static unsigned long stableAltitudeTime = 0;
    static float lastAltitude = -9999;
    static bool stableAltitudeDetected = false;
    const float LANDING_ALTITUDE_THRESHOLD = 2;
    const unsigned long STABLE_TIME_REQUIRED = 5000;

    if (lastAltitude == -9999) {
        lastAltitude = altitude;
        return false;
    }

    if (abs(altitude - lastAltitude) < LANDING_ALTITUDE_THRESHOLD) {
        if (!stableAltitudeDetected) {
            stableAltitudeDetected = true;
            stableAltitudeTime = millis();
            Serial.println("Stable altitude detected, potential landing");
        }

        if (millis() - stableAltitudeTime >= STABLE_TIME_REQUIRED) {
            Serial.println("LANDING CONFIRMED!");

            blueLed.state = DOUBLE_BLINK;
            greenLed.state = DOUBLE_BLINK;
            buzzerState.pattern = LANDED_BEEP;

            hasRocketLanded = true;

            return true;
        }
    } else {
        stableAltitudeDetected = false;
    }

    lastAltitude = altitude;
    return false;
}

void Rocket::landed(){
    greenLed.state = DOUBLE_BLINK;
    buzzerState.pattern = LANDED_BEEP;
    blueLed.state = ON;

    logFile.close();
    SD.end();

    sendTelemetry();
}

bool Rocket::houstonWeHaveAProblem(const String &errorMessage) {
    //TODO: Send error message to the control panel
    buzzerState.pattern = ERROR_BEEP;

    redLed.state = BLINK;
    blueLed.state = OFF;

    logData(errorMessage);

    return false;
}

void Rocket::setupLeds() {
    pinMode(blueLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);

    blueLed.pin = blueLedPin;
    greenLed.pin = greenLedPin;
    redLed.pin = redLedPin;
}

void Rocket::updateBuzzer() {
    const unsigned long shortBeep = 100;
    const unsigned long longBeep = 200;
    const unsigned long intervalSOS = 50;

    if (buzzerState.pattern != buzzerState.previousPattern) {
        buzzerState.previousPattern = buzzerState.pattern;
        buzzerState.previousMillis = millis();
    }

    switch(buzzerState.pattern) {
        case SILENT:
            ledcWriteTone(0, 0);
            break;

        case SINGLE_BEEP:
            if(buzzerState.beepCounter == 0) {
                ledcWriteTone(0, 2000);
                buzzerState.previousMillis = millis();
                buzzerState.beepCounter++;
            }
            if(millis() - buzzerState.previousMillis >= shortBeep) {
                ledcWriteTone(0, 0);
                buzzerState.pattern = SILENT;
                buzzerState.beepCounter = 0;
            }
            break;

        case CONTINUOUS_BEEP:
            if(millis() - buzzerState.previousMillis >= shortBeep) {
                buzzerState.buzzerState = !buzzerState.buzzerState;
                buzzerState.previousMillis = millis();
                buzzerState.buzzerState ? ledcWriteTone(0, 1500) : ledcWriteTone(0, 0);
            }
            break;

        case DOUBLE_BEEP:
            if(millis() - buzzerState.previousMillis >= (buzzerState.buzzerState ? shortBeep : longBeep)) {
                buzzerState.buzzerState = !buzzerState.buzzerState;
                buzzerState.previousMillis = millis();
                if(buzzerState.buzzerState) {
                    ledcWriteTone(0, 1500);
                    buzzerState.beepCounter++;
                } else {
                    ledcWriteTone(0, 0);
                    if(buzzerState.beepCounter >= 2) {
                        buzzerState.pattern = SILENT;
                        buzzerState.beepCounter = 0;
                    }
                }
            }
            break;

        case ERROR_BEEP:
            if(millis() - buzzerState.previousMillis >= (buzzerState.buzzerState ? shortBeep : shortBeep)) {
                buzzerState.buzzerState = !buzzerState.buzzerState;
                buzzerState.previousMillis = millis();
                if(buzzerState.buzzerState) {
                    ledcWriteTone(0, 1000);
                    buzzerState.beepCounter++;
                } else {
                    ledcWriteTone(0, 0);
                    if(buzzerState.beepCounter >= 3) {
                        buzzerState.pattern = SILENT;
                        buzzerState.beepCounter = 0;
                    }
                }
            }
            break;

        case SOS:
            if(millis() - buzzerState.previousMillis >= intervalSOS) {
                switch(buzzerState.beepCounter) {
                    case 0: case 1: case 2: // 3 beeps cortos
                        ledcWriteTone(0, 1500);
                        buzzerState.previousMillis = millis();
                        buzzerState.beepCounter++;
                        break;
                    case 3: case 5: case 7: // Silencios entre beeps
                        ledcWriteTone(0, 0);
                        buzzerState.previousMillis = millis();
                        buzzerState.beepCounter++;
                        break;
                    case 4: case 6: // 3 beeps largos
                        ledcWriteTone(0, 1500);
                        buzzerState.previousMillis = millis();
                        buzzerState.beepCounter++;
                        break;
                    case 8: // Silencio final
                        ledcWriteTone(0, 0);
                        buzzerState.pattern = SILENT;
                        buzzerState.beepCounter = 0;
                        break;
                }
            }
            break;

        case ASCENT_BEEP:
            buzzerState.beepCounter = 0;
//            if(millis() - buzzerState.previousMillis >= (buzzerState.buzzerState ? 50 : 100)) {
//                buzzerState.buzzerState = !buzzerState.buzzerState;
//                buzzerState.previousMillis = millis();
//                buzzerState.buzzerState ? tone(buzzerPin, 2500) : noTone(buzzerPin);
//            }
            break;

        case DESCENT_BEEP:
            if(millis() - buzzerState.previousMillis >= (buzzerState.buzzerState ? shortBeep : longBeep)) {
                buzzerState.buzzerState = !buzzerState.buzzerState;
                buzzerState.previousMillis = millis();
                if(buzzerState.buzzerState) {
                    ledcWriteTone(0, 1500);
                    buzzerState.beepCounter++;
                } else {
                    ledcWriteTone(0, 0);
                    if(buzzerState.beepCounter >= 2) {
                        buzzerState.pattern = SILENT;
                        buzzerState.beepCounter = 0;
                    }
                }
            }
            break;

        case LANDED_BEEP:
            if(millis() - buzzerState.previousMillis >= (buzzerState.buzzerState ? 1000 : 5000)) {
                buzzerState.buzzerState = !buzzerState.buzzerState;
                buzzerState.previousMillis = millis();
                buzzerState.buzzerState ? ledcWriteTone(0, 1200) : ledcWriteTone(0, 0);
            }
            break;
    }
}

void Rocket::updateLeds() {
    updateLed(blueLed);
    updateLed(greenLed);
    updateLed(redLed);
}

void Rocket::updateLed(Led &led) {
    if (led.state != led.previousState) {
        led.previousState = led.state;
        led.previousMillis = millis();
    }

    switch(led.state) {
        case OFF:
            digitalWrite(led.pin, LOW);
            break;
        case ON:
            digitalWrite(led.pin, HIGH);
            break;
        case BLINK:
            if(millis() - led.previousMillis >= 250) {
                led.ledState = !led.ledState;
                led.previousMillis = millis();
                digitalWrite(led.pin, led.ledState);
            }
            break;
        case DOUBLE_BLINK:
            if(millis() - led.previousMillis >= 100) {
                led.blinkCounter++;
                if(led.blinkCounter % 2 == 0) {
                    led.ledState = !led.ledState;
                    led.previousMillis = millis();
                    digitalWrite(led.pin, led.ledState);
                }
//                if(led.blinkCounter >= 4) {
//                    led.state = OFF;
//                    led.blinkCounter = 0;
//                }
            }
            break;
    }
}

void Rocket::resetLeds() {
    blueLed.state = OFF;
    greenLed.state = ON;
    redLed.state = OFF;
    buzzerState.pattern = SINGLE_BEEP;
}

bool Rocket::initializeLogging() {
    logFileName = "/flight_";

    if (gpsDateTime != "") {
        String formattedDateTime = gpsDateTime;
        formattedDateTime.replace("/", "");
        formattedDateTime.replace(":", "");
        formattedDateTime.replace(" ", "_");
        logFileName += formattedDateTime;
    } else {
        int fileCounter = 0;
        while (SD.exists(logFileName + String(fileCounter) + ".csv")) {
            fileCounter++;
        }
        logFileName += String(fileCounter);
    }

    logFileName += ".csv";

    logFile = SD.open(logFileName, FILE_WRITE);

    if (!logFile) {
        Serial.println("Error al crear el archivo de registro: " + logFileName);
        return false;
    }

    logFile.println("Timestamp,Event,Temperature,Pressure,Altitude,Humidity,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Latitude,Longitude,DateTime,DS18B20Temperature");
    logFile.flush();

    Serial.println("Registro inicializado: " + logFileName);
    return true;
}

void Rocket::logData(const String &message) {
    if (!logFile) {
        return;
    }

    unsigned long timestamp = millis();

    logFile.print(timestamp);
    logFile.print(",");
    logFile.print(message);
    logFile.println();

    logFile.flush();

    Serial.print("[LOG] ");
    Serial.print(timestamp);
    Serial.print(": ");
    Serial.println(message);
}

void Rocket::logSensorData() {
    if (!logFile) {
        return;
    }

    unsigned long timestamp = millis();

    logFile.print(timestamp);
    logFile.print(",SensorData,");
    logFile.print(temperature);
    logFile.print(",");
    logFile.print(pressure);
    logFile.print(",");
    logFile.print(altitude);
    logFile.print(",");
    logFile.print(humidity);
    logFile.print(",");
    logFile.print(accelerometer.acceleration.x);
    logFile.print(",");
    logFile.print(accelerometer.acceleration.y);
    logFile.print(",");
    logFile.print(accelerometer.acceleration.z);
    logFile.print(",");
    logFile.print(gyroscope.gyro.x);
    logFile.print(",");
    logFile.print(gyroscope.gyro.y);
    logFile.print(",");
    logFile.print(gyroscope.gyro.z);
    logFile.print(",");
    logFile.print(gpsLatitude, 6);
    logFile.print(",");
    logFile.print(gpsLongitude, 6);
    logFile.print(",");
    logFile.print(gpsDateTime);
    logFile.print(",");
    logFile.print(ds18b20Temperature);
    logFile.println();

    logFile.flush();
}

bool Rocket::initializeRF24() {
    logData("[INFO] Initializing RF24 communication...");

    // Initialize RF24 on HSPI bus
    if (!radio.begin(&hspi)) {
        Serial.println("Error initializing RF24");
        return houstonWeHaveAProblem("[ERROR] Error initializing RF24");
    }

    // Configure RF24
    radio.setPALevel(RF24_PA_HIGH);      // Set power amplifier level (LOW, HIGH, MAX)
    radio.setDataRate(RF24_1MBPS);       // Set data rate (250KBPS, 1MBPS, 2MBPS)
    radio.setChannel(11);                // Set RF channel (0-125)
    radio.setRetries(3, 5);              // Set retries (delay, count)
    radio.setCRCLength(RF24_CRC_16);     // Set CRC length

    // Open pipes for communication
    radio.openWritingPipe(00002);
    radio.openReadingPipe(1, 00001);

    //TODO A saber porque cojones no funciona el ack
    radio.setAutoAck(false);

    // Start listening
    radio.startListening();

    logData("[INFO] RF24 initialized successfully");
    Serial.println("RF24 initialized successfully");

    return true;
}

void Rocket::receiveCommands() {
    radio.startListening();

    // Check for incoming data without blocking
    if (radio.available()) {
//        char text[32] = "";
//        radio.read(&text, sizeof(text));
//        Serial.print("Mensaje recibido: ");
//        Serial.println(text);

        // Read command data
        CommandData receivedCommand;
        radio.read(&receivedCommand, sizeof(CommandData));

        logData("[INFO] Command received: " + String(receivedCommand.command));

        // Process the command
        if (processCommand(receivedCommand)) {
            logData("[INFO] Command processed: " + String(receivedCommand.command));
        } else {
            logData("[WARNING] Invalid command received: " + String(receivedCommand.command));
        }
    }

//    radio.stopListening();
}

void Rocket::sendTelemetry() {
    // Update telemetry data with current sensor readings
    telemetryData.altitude = altitude;
    telemetryData.temperature = temperature;
    telemetryData.pressure = pressure;
    telemetryData.accelerationX = accelerometer.acceleration.x;
    telemetryData.accelerationY = accelerometer.acceleration.y;
    telemetryData.accelerationZ = accelerometer.acceleration.z;
    telemetryData.gyroX = gyroscope.gyro.x;
    telemetryData.gyroY = gyroscope.gyro.y;
    telemetryData.gyroZ = gyroscope.gyro.z;
    telemetryData.latitude = gpsLatitude;
    telemetryData.longitude = gpsLongitude;
    telemetryData.parachuteDeployed = parachuteDeployed;
    telemetryData.hasReachedApogee = hasReachedApogee;
    telemetryData.timestamp = millis();
    telemetryData.landed = hasRocketLanded;

    // Stop listening so we can transmit
    radio.stopListening();

    // Send telemetry data
    bool success = radio.write(&telemetryData, sizeof(TelemetryData));

    if (success) {
        // Log successful transmission at a reasonable interval (not every time)
        static unsigned long lastTransmissionLog = 0;
        if (millis() - lastTransmissionLog > 100) {  // Log every 5 seconds
            logData("[INFO] Telemetry transmitted successfully");
            lastTransmissionLog = millis();
        }
    } else {
        // Log failed transmission
        logData("[WARNING] Failed to transmit telemetry data");

        Serial.println("Failed to transmit telemetry data");
        Serial.print("Power Level: "); Serial.println(radio.getPALevel());
        Serial.print("Data Rate: "); Serial.println(radio.getDataRate());
        Serial.print("Channel: "); Serial.println(radio.getChannel());
    }

    // Resume listening
    radio.startListening();
}

bool Rocket::sendRocketReady(){
    radio.stopListening();

    CommandData command;
    command.command = CMD_ROCKET_READY;
    command.parameter = 0;

    bool success = radio.write(&command, sizeof(CommandData));

    if (success) {
        logData("[INFO] Rocket ready transmitted successfully");
    } else {
        logData("[WARNING] Failed to transmit rocket ready data");
    }

    radio.startListening();

    return success;
}

bool Rocket::processCommand(CommandData& command) {
    switch (command.command) {
        case CMD_PING:
            // Acknowledge ping (no specific action needed)
            logData("[INFO] Ping received from control panel");
            return true;

        case CMD_DEPLOY_PARACHUTE:
            // Manual parachute deployment
            if (!parachuteDeployed) {
                deployParachute();
                logData("[INFO] Parachute manually deployed via control panel command");
            } else {
                logData("[INFO] Parachute deployment command received but parachute already deployed");
            }
            return true;

        case CMD_ABORT:
            // Emergency abort
            logData("[WARNING] ABORT command received from control panel");
            if (!parachuteDeployed) {
                deployParachute();
                logData("[INFO] Parachute deployed during ABORT sequence");
            }

            // Set LEDs and buzzer to error state
            houstonWeHaveAProblem("[WARNING] ABORT command received from control panel");

            return true;

        case CMD_REBOOT:
            // Reboot the system
            logData("[WARNING] Reboot command received from control panel");
            delay(100);  // Small delay to ensure log is written
            ESP.restart();
            return true;

        case CMD_WAKE_UP:
            // Wake up from sleep mode
            receiveWakeUp = true;
            logData("[INFO] Wake up command received from control panel");
            return true;

        case CMD_LAUNCH:
            // Lunch the rocket
            logData("[INFO] Lunch command received from control panel");
            receiveLunch = true;
            return true;

        default:
            // Unknown command
            return false;
    }
}

bool Rocket::hasReceivedWakeUp(){
    return receiveWakeUp;
}

bool Rocket::hasReceivedLunch(){
    return receiveLunch;
}