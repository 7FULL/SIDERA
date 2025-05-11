/**
 * LoRa Communication System Implementation
 */

#include "LoRaSystem.h"
#include "Config.h"
#include "States.h"

static const size_t MAX_CHUNK_SIZE = 16;

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

    if (!LoRa.begin(868E6)) { // Default to 915E6 for US region 868E6 for EU
        Serial.println("LoRa: ERROR: Initialization failed!");
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    // Set parameters for better performance
    LoRa.setSpreadingFactor(12);      // Range 6-12, higher = more range but slower
    LoRa.setSignalBandwidth(62.5E3);  // 125kHz bandwidth
    LoRa.setCodingRate4(8);          // 4/5 coding rate
    LoRa.setPreambleLength(12);       // Default preamble length
    LoRa.setSyncWord(0x34);
    LoRa.enableCrc();                // Enable CRC checking
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN); // Higher power

    // Set callback for received packets
    LoRa.onReceive(onReceiveStatic);

    // Start listening for packets
//    LoRa.receive();

    Serial.println("LoRa: Module initialized successfully!");

    status = SensorStatus::OK;
    lastReadingTime = millis();
    return status;
}

void LoRaSystem::onReceiveStatic(int packetSize) {

    Serial.println("LoRa: Packet received!");
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
        Serial.println("LoRa: System not operational");
        return false;
    }

    RocketState currentState = storageManager->getCurrentState();

    bool canProcessCommands = (currentState == RocketState::INIT ||
                               currentState == RocketState::GROUND_IDLE ||
                               currentState == RocketState::READY);

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Starting packet...");
    #endif

    if (!LoRa.beginPacket()) {
        Serial.println("LoRa: Failed to start packet");

        //We try putting it in idle mode and then starting again
        LoRa.idle();
        delay(10);
        if (!LoRa.beginPacket()) {
            Serial.println("LoRa: Failed to start packet after idle");
            return false;
        }else{
            #ifdef ENABLE_LORA_DEBUG
            Serial.println("LoRa: Packet started successfully after idle");
            #endif
            receiving = false;
        }
    }else{
        #ifdef ENABLE_LORA_DEBUG
        Serial.println("LoRa: Packet started successfully");
        #endif
    }

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Writing header...");
    #endif

//    LoRa.write(destinationId);

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Writing node ID...");
    #endif

//    LoRa.write(nodeId);

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Writing packet counter...");
    #endif

//    LoRa.write(packetCounter++);

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Writing message type...");
    #endif

//    LoRa.write(static_cast<uint8_t>(message.type));

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Writing message length...");
    #endif

    LoRa.print("Hello World #");

//    if (message.data != nullptr && message.length > 0) {
//        #ifdef ENABLE_LORA_DEBUG
//        Serial.println("LoRa: Condition OK, entering data-write block");
//        Serial.print("LoRa: Writing %d bytes of data...\n");
//        Serial.println(message.length);
//        #endif
//
//        #ifdef ENABLE_LORA_DEBUG
//        Serial.println("LoRa: About to write payload in chunks...");
//        #endif
//
//        size_t bytesRemaining = message.length;
//        const uint8_t* bufferPtr = reinterpret_cast<const uint8_t*>(message.data);
//
//        while (bytesRemaining > 0) {
//            size_t chunkSize = (bytesRemaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : bytesRemaining;
//
//            #ifdef ENABLE_LORA_DEBUG
//            Serial.print("LoRa: Writing chunk of ");
//            Serial.print(chunkSize);
//            Serial.println(" bytes...");
//            #endif
//
//            LoRa.write(bufferPtr, chunkSize);
//
//            #ifdef ENABLE_LORA_DEBUG
//            Serial.println("LoRa: Chunk written");
//            #endif
//
//            bufferPtr     += chunkSize;
//            bytesRemaining -= chunkSize;
//
//            // Pequeña pausa para dar tiempo al módulo a procesar el chunk
//            delay(5);
//        }
//
//        #ifdef ENABLE_LORA_DEBUG
//        Serial.println("LoRa: Finished writing all chunks");
//        #endif
//    } else {
//        Serial.println("LoRa: No data to send");
//        return false;
//    }

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Ending packet...");
    #endif

//    LoRa.endPacket(true);

    LoRa.endPacket(true);

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Waiting for transmission to complete...");
    #endif
    while (LoRa.isTransmitting()) {
        delay(10);
    }

    #ifdef ENABLE_LORA_DEBUG
    Serial.println("LoRa: Transmission complete");
    Serial.println("LoRa: Packet sent successfully");
    #endif

    if (storageManager) {
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg),
                 "LoRa: Sent %d byte message to ID %d, type %d",
                 message.length, destinationId, static_cast<uint8_t>(message.type));
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::COMMUNICATION, logMsg);

        if (canProcessCommands) {
            #ifdef ENABLE_LORA_DEBUG
            Serial.println("LoRa: Changing to receive mode...");
            #endif
//            while (LoRa.isTransmitting()) {
//                delay(10);
//            }
            LoRa.receive();
            receiving = true;

            #ifdef ENABLE_LORA_DEBUG
            Serial.println("LoRa: Now in receive mode");
            #endif
        }
    }

    #ifdef ENABLE_LORA_DEBUG
    Serial.print("LoRa: Packet counter: ");
    Serial.println(packetCounter);
    Serial.print("LoRa: Destination ID: ");
    Serial.println(destinationId);
    Serial.print("LoRa: Node ID: ");
    Serial.println(nodeId);
    Serial.print("LoRa: Message type: ");
    Serial.println(static_cast<uint8_t>(message.type));
    Serial.print("LoRa: Message length: ");
    Serial.println(message.length);
//    Serial.print("LoRa: Message data: ");
//    for (size_t i = 0; i < message.length; ++i) {
//        Serial.print(message.data[i], HEX);
//        Serial.print(" ");
//    }
    Serial.println();
    #endif

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