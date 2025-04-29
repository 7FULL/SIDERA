/**
 * LoRa Communication System Implementation
 */

#include "LoRaSystem.h"

// Initialize static instance pointer
LoRaSystem* LoRaSystem::instance = nullptr;

LoRaSystem::LoRaSystem(SPIClass& spi, int8_t csPin, int8_t resetPin, int8_t irqPin, StorageManager* storageManager)
        : spi(spi), csPin(csPin), resetPin(resetPin), irqPin(irqPin), storageManager(storageManager) {
    instance = this;
}

LoRaSystem::~LoRaSystem() {
    // Clean up LoRa
    LoRa.end();
    if (instance == this) {
        instance = nullptr;
    }
}

SensorStatus LoRaSystem::begin() {
    // Configure LoRa module with SPI and pins
    LoRa.setPins(csPin, resetPin, irqPin);
    LoRa.setSPI(spi);

    // We wait for the LoRa module to be ready
    Serial.println("LoRa: Waiting for module to be ready...");
    //Number of attempts to wait for the module
    int maxAttempts = 10;
    int attempts = 0;
    //Auxiliary variable to check if the module is ready
    bool moduleReady = false;
    while (attempts < maxAttempts) {
        Serial.print("Attempt ");
        Serial.print(attempts + 1);
        Serial.print(" of ");
        Serial.print(maxAttempts);

        // Check if the module is ready
        if (LoRa.begin(868E6)) { // Default to 915 MHz for US region 868E6 for EU
            moduleReady = true;
            break;
        }
        attempts++;
        delay(1000); // Wait before retrying
    }

    if (!moduleReady) {
        Serial.println("LoRa: Module not found!");
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    // Set parameters for better performance
    LoRa.setSpreadingFactor(7);      // 7 is default
    LoRa.setSignalBandwidth(125E3);  // 125 kHz is typical
    LoRa.setCodingRate4(5);          // 4/5 coding rate
    LoRa.setSyncWord(0x34);          // Default sync word
    LoRa.enableCrc();                // Enable CRC checking

    // Set callback for received packets
    LoRa.onReceive(onReceiveStatic);

    // Start listening for packets
    LoRa.receive();

    Serial.println("LoRa: Module initialized successfully!");

    status = SensorStatus::OK;
    lastReadingTime = millis();
    return status;
}

void LoRaSystem::onReceiveStatic(int packetSize) {
    if (instance) {
        instance->processPacket(packetSize);
    }
}

SensorStatus LoRaSystem::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

    // Nothing to do in update - LoRa interrupt will handle incoming packets
    // Just update the last reading time
    lastReadingTime = millis();
    return status;
}

bool LoRaSystem::isOperational() {
    return status == SensorStatus::OK;
}

SensorStatus LoRaSystem::getStatus() const {
    return status;
}

const char* LoRaSystem::getName() const {
    return "SX1276 LoRa";
}

unsigned long LoRaSystem::getLastReadingTime() const {
    return lastReadingTime;
}

bool LoRaSystem::sendMessage(const Message& message) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Start LoRa packet
    LoRa.beginPacket();

    // Add header with destination and source IDs
    LoRa.write(destinationId);
    LoRa.write(nodeId);
    LoRa.write(packetCounter++);
    LoRa.write(static_cast<uint8_t>(message.type));

    // Add message data if available
    if (message.data != nullptr && message.length > 0) {
        LoRa.write(message.data, message.length);
    }

    // End and send packet
    bool result = LoRa.endPacket();

    // Return to receive mode
    LoRa.receive();

    if (storageManager && result) {
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg), "LoRa: Sent %d byte message, type %d",
                 message.length, static_cast<int>(message.type));
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::COMMUNICATION, logMsg);
    }

    return result;
}

bool LoRaSystem::hasReceivedMessages() {
    return !receivedMessages.empty();
}

bool LoRaSystem::getNextMessage(Message& message) {
    if (receivedMessages.empty()) {
        return false;
    }

    message = receivedMessages.front();
    receivedMessages.pop();
    return true;
}

void LoRaSystem::setNodeId(uint8_t id) {
    nodeId = id;
}

uint8_t LoRaSystem::getNodeId() const {
    return nodeId;
}

void LoRaSystem::setDestinationId(uint8_t id) {
    destinationId = id;
}

int LoRaSystem::getLastRssi() {
    return lastRssi;
}

float LoRaSystem::getLastSnr() {
    return lastSnr;
}

bool LoRaSystem::setTxPower(int8_t power) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setTxPower(power);
    return true;
}

bool LoRaSystem::enableLowPowerMode() {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Put LoRa module into sleep mode
    LoRa.sleep();
    lowPowerMode = true;
    return true;
}

bool LoRaSystem::disableLowPowerMode() {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Wake up LoRa module and put back into receive mode
    LoRa.idle();
    LoRa.receive();
    lowPowerMode = false;
    return true;
}

bool LoRaSystem::setFrequency(long frequency) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setFrequency(frequency);
    return true;
}

bool LoRaSystem::setSpreadingFactor(int sf) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setSpreadingFactor(sf);
    return true;
}

bool LoRaSystem::setSignalBandwidth(long bandwidth) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setSignalBandwidth(bandwidth);
    return true;
}

bool LoRaSystem::setCodingRate(int denominator) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setCodingRate4(denominator);
    return true;
}

bool LoRaSystem::enableCrc() {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.enableCrc();
    return true;
}

bool LoRaSystem::disableCrc() {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.disableCrc();
    return true;
}

void LoRaSystem::processPacket(int packetSize) {
    if (packetSize == 0 || status != SensorStatus::OK) {
        return;
    }

    // Store RSSI and SNR
    lastRssi = LoRa.packetRssi();
    lastSnr = LoRa.packetSnr();

    // Read packet header
    uint8_t destination = LoRa.read();
    uint8_t source = LoRa.read();
    uint8_t counter = LoRa.read();
    uint8_t type = LoRa.read();

    // Check if this packet is for us
    if (destination != nodeId && destination != 0) {
        return; // Not for us
    }

    // Create message structure
    Message message;
    message.type = static_cast<MessageType>(type);
    message.priority = 0; // Default priority
    message.timestamp = millis();

    // Read payload data (remaining bytes)
    message.length = packetSize - 4; // Subtract header size

    if (message.length > 0) {
        message.data = new uint8_t[message.length];
        for (int i = 0; i < message.length && LoRa.available(); i++) {
            message.data[i] = LoRa.read();
        }
    } else {
        message.data = nullptr;
    }

    // Add to received messages queue
    receivedMessages.push(message);

    // Log received message
    if (storageManager) {
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg),
                 "LoRa: Received %d byte message from ID %d, type %d, RSSI %d",
                 message.length, source, type, lastRssi);
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::COMMUNICATION, logMsg);
    }
}