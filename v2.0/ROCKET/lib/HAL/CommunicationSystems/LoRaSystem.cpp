/**
 * LoRa Communication System Implementation
 */

#include "LoRaSystem.h"
#include "Config.h"
#include "States.h"

static const size_t MAX_CHUNK_SIZE = 16;

//// Initialize static instance pointer
//LoRaSystem* LoRaSystem::instance = nullptr;

LoRaSystem::LoRaSystem(SPIClass& spi, int8_t csPin, int8_t resetPin, int8_t irqPin, StorageManager* storageManager)
        : spi(spi), csPin(csPin), resetPin(resetPin), irqPin(irqPin), storageManager(storageManager) {
//    instance = this;
}

LoRaSystem::~LoRaSystem() {
    // Clean up LoRa
    LoRa.end();
//    if (instance == this) {
//        instance = nullptr;
//    }
}

SensorStatus LoRaSystem::begin() {
    LoRa.setPins(csPin, resetPin, irqPin);
    LoRa.setSPI(spi);

    if (resetPin >= 0) {
        Serial.println("LoRa: Performing hardware reset...");
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, LOW);
        delay(20);
        digitalWrite(resetPin, HIGH);
        delay(150); // Give the module more time to stabilize
    }

    if (!LoRa.begin(LORA_FREQUENCY)) { // Default to 915E6 for US region 868E6 for EU
        Serial.println("LoRa: ERROR: Initialization failed!");
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    // Set parameters for better performance
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);      // Range 6-12, higher = more range but slower
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);  // 125kHz bandwidth
    LoRa.setCodingRate4(LORA_CODING_RATE);          // 4/5 coding rate
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);       // Default preamble length
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();                // Enable CRC checking
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN); // Higher power

    // Set callback for received packets
//    LoRa.onReceive(onReceiveStatic);

    // Start listening for packets
//    LoRa.receive();

    Serial.println("LoRa: Module initialized successfully!");

    status = SensorStatus::OK;
    lastReadingTime = millis();
    return status;
}

SensorStatus LoRaSystem::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

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
        Serial.println("LoRa: System not operational");
        return false;
    }

    LoRa.idle();

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Starting packet...");
    #endif

    if (!LoRa.beginPacket()) {
        Serial.println("LoRa: Failed to start packet");
        return false;
    }

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Packet started successfully");
    #endif

    LoRa.write(destinationId);
    LoRa.write(nodeId);
    LoRa.write(packetCounter++);
    LoRa.write(static_cast<uint8_t>(message.type));

    if (message.data != nullptr && message.length > 0) {
        // Write the message data
        LoRa.write(message.data, message.length);
    } else {
        Serial.println("LoRa: No data to send");
        return false;
    }

    LoRa.endPacket();

    if (storageManager) {
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::COMMUNICATION, "LoRa: Message sent");
    }

    return true;
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
        Serial.println("LoRa: Invalid packet or system not operational");
        return;
    }

    // Store RSSI and SNR
    lastRssi = LoRa.packetRssi();
    lastSnr = LoRa.packetSnr();

    // Check if this is a simple format packet from the control panel
    if (packetSize >= 4 && packetSize < 10) {
        // Read the simple packet format
        uint8_t destination = LoRa.read();
        uint8_t source = LoRa.read();
        uint8_t counter = LoRa.read();
        uint8_t cmdType = LoRa.read();

        // Check if this packet is for us
        if (destination != nodeId && destination != 0) {
            Serial.println("LoRa: Packet not addressed to us, ignoring");
            return; // Not for us
        }

        Serial.printf("LoRa: Received simple command packet - Type: %d\n", cmdType);

        // Create a message with the command data
        Message message;
        message.type = MessageType::COMMAND_RESPONSE;
        message.priority = 200; // High priority for commands
        message.timestamp = millis();

        // Create a simple payload with just the command type
        // This allows the CommandHandler to process it
        message.length = 1;
        message.data = new uint8_t[1];
        message.data[0] = cmdType;

        // Add to received messages queue
        receivedMessages.push(message);

        Serial.print("LoRa: Command added to queue, queue size now: ");
        Serial.println(receivedMessages.size());

        // Log received command
        if (storageManager) {
            char logMsg[64];
            snprintf(logMsg, sizeof(logMsg),
                     "LoRa: Received simple command %d from ID %d",
                     cmdType, source);
            storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION, logMsg);
        }
    }
    else {
        // This could be a standard protocol packet
        // Read the full packet into a buffer
        std::vector<uint8_t> buffer;

        while (LoRa.available()) {
            buffer.push_back(LoRa.read());
        }

        // Create message structure
        Message message;
        message.type = MessageType::COMMAND_RESPONSE;
        message.priority = 100;
        message.timestamp = millis();
        message.length = buffer.size();

        if (!buffer.empty()) {
            message.data = new uint8_t[buffer.size()];
            memcpy(message.data, buffer.data(), buffer.size());
        }
        else {
            message.data = nullptr;
            message.length = 0;
        }

        // Add to received messages queue
        receivedMessages.push(message);
    }

    Serial.println("LoRa: Packet processing complete");
}