/**
 * LoRa Communication System Implementation
 */

#include "LoRaSystem.h"

LoRaSystem* LoRaSystem::instance = nullptr;

LoRaSystem::LoRaSystem(SPIClass& spi, int8_t csPin, int8_t resetPin, int8_t irqPin, int8_t txEnPin, int8_t rxEnPin)
        : spi(spi), csPin(csPin), resetPin(resetPin), irqPin(irqPin), txEnPin(txEnPin), rxEnPin(rxEnPin) {
    instance = this;
}

LoRaSystem::~LoRaSystem() {
    // Cleanup resources
    LoRa.end();
    if (instance == this) {
        instance = nullptr;
    }
}

SensorStatus LoRaSystem::begin() {
    // Set up SPI for LoRa module
    LoRa.setSPI(spi);
    LoRa.setPins(csPin, resetPin, irqPin);

    // Set up Tx/Rx enable pins if provided
    if (txEnPin >= 0) {
        pinMode(txEnPin, OUTPUT);
        digitalWrite(txEnPin, HIGH); // Enable Tx
    }

    if (rxEnPin >= 0) {
        pinMode(rxEnPin, OUTPUT);
        digitalWrite(rxEnPin, HIGH); // Enable Rx
    }

    // Initialize LoRa
    if (!LoRa.begin(915E6)) { // Default to 915 MHz
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Configure LoRa parameters
    LoRa.setSignalBandwidth(125E3);    // 125 kHz bandwidth
    LoRa.setSpreadingFactor(7);        // SF7
    LoRa.setCodingRate4(5);            // 4/5 coding rate
    LoRa.enableCrc();                  // Enable CRC
    LoRa.setTxPower(17);               // 17 dBm output power

    // Set up callback for received packets
    LoRa.onReceive(onReceiveStatic);

    LoRa.receive(); // Put into continuous receive mode

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

    // Check for any received packets (handled by callback)

    // Retransmit any pending packets that need acknowledgement
    retransmitPendingPackets();

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
    return "LoRa SX1276";
}

unsigned long LoRaSystem::getLastReadingTime() const {
    return lastReadingTime;
}

bool LoRaSystem::sendMessage(const Message& message) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Convert message to LoRa packet
    LoRaPacket packet = messageToPacket(message);

    // Send the packet
    return sendPacket(packet);
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

    // Disable Tx/Rx if pins are provided
    if (txEnPin >= 0) {
        digitalWrite(txEnPin, LOW);
    }

    if (rxEnPin >= 0) {
        digitalWrite(rxEnPin, LOW);
    }

    lowPowerMode = true;
    return true;
}

bool LoRaSystem::disableLowPowerMode() {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Enable Tx/Rx if pins are provided
    if (txEnPin >= 0) {
        digitalWrite(txEnPin, HIGH);
    }

    if (rxEnPin >= 0) {
        digitalWrite(rxEnPin, HIGH);
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

bool LoRaSystem::setSignalBandwidth(long bandwidth) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setSignalBandwidth(bandwidth);
    return true;
}

bool LoRaSystem::setSpreadingFactor(int sf) {
    if (status != SensorStatus::OK) {
        return false;
    }

    LoRa.setSpreadingFactor(sf);
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

    // Read packet header
    uint8_t destination = LoRa.read();
    uint8_t source = LoRa.read();
    uint8_t id = LoRa.read();
    uint8_t flags = LoRa.read();
    uint8_t messageTypeValue = LoRa.read();
    MessageType messageType = static_cast<MessageType>(messageTypeValue);
    uint16_t length = LoRa.read() << 8 | LoRa.read(); // 16-bit length (MSB first)

    // Check if this packet is for us
    if (destination != nodeId && destination != 0) {
        return; // Not for us
    }

    // Read payload
    std::vector<uint8_t> payload;
    payload.reserve(length);

    for (uint16_t i = 0; i < length && LoRa.available(); i++) {
        payload.push_back(LoRa.read());
    }

    // Create packet
    LoRaPacket packet;
    packet.destination = destination;
    packet.source = source;
    packet.id = id;
    packet.flags = flags;
    packet.type = messageType;
    packet.length = length;
    packet.payload = payload;

    // Save RSSI and SNR
    lastRssi = LoRa.packetRssi();
    lastSnr = LoRa.packetSnr();

    // Handle acknowledgements
    if (flags & LoRaPacket::FLAG_ACK) {
        handleAcknowledgement(packet);
        return; // ACK packets don't contain application data
    }

    // Send ACK if requested
    if (flags & LoRaPacket::FLAG_ACK_REQUEST) {
        // Create ACK packet
        LoRaPacket ackPacket;
        ackPacket.destination = source;
        ackPacket.source = nodeId;
        ackPacket.id = id; // Same ID as the original packet
        ackPacket.flags = LoRaPacket::FLAG_ACK;
        ackPacket.type = messageType;
        ackPacket.length = 0;

        // Send ACK packet
        sendPacket(ackPacket);
    }

    // Convert packet to message and add to queue
    Message message = packetToMessage(packet);
    receivedMessages.push(message);
}

bool LoRaSystem::sendPacket(const LoRaPacket& packet) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Exit receive mode
    LoRa.idle();

    // Begin packet
    LoRa.beginPacket();

    // Write header
    LoRa.write(packet.destination);
    LoRa.write(packet.source);
    LoRa.write(packet.id);
    LoRa.write(packet.flags);
    LoRa.write(static_cast<uint8_t>(packet.type));
    LoRa.write(packet.length >> 8);    // Length MSB
    LoRa.write(packet.length & 0xFF);  // Length LSB

    // Write payload
    for (uint8_t b : packet.payload) {
        LoRa.write(b);
    }

    // End packet and send
    bool result = LoRa.endPacket();

    // Add to sent packets if waiting for ACK
    if (result && (packet.flags & LoRaPacket::FLAG_ACK_REQUEST)) {
        sentPackets.push_back(packet);
    }

    // Return to receive mode
    LoRa.receive();

    return result;
}

void LoRaSystem::handleAcknowledgement(const LoRaPacket& ackPacket) {
    // Find the original packet and remove it from the pending list
    for (auto it = sentPackets.begin(); it != sentPackets.end(); ++it) {
        if (it->id == ackPacket.id && it->destination == ackPacket.source) {
            sentPackets.erase(it);
            break;
        }
    }
}

LoRaPacket LoRaSystem::messageToPacket(const Message& message) {
    LoRaPacket packet;
    packet.destination = destinationId;
    packet.source = nodeId;
    packet.id = nextPacketId++;
    packet.flags = LoRaPacket::FLAG_ACK_REQUEST; // Request ACK for all packets
    packet.type = message.type;
    packet.length = message.length;

    // Copy payload
    packet.payload.resize(message.length);
    for (uint16_t i = 0; i < message.length; i++) {
        packet.payload[i] = message.data[i];
    }

    return packet;
}

Message LoRaSystem::packetToMessage(const LoRaPacket& packet) {
    Message message;
    message.type = packet.type;
    message.priority = 0; // Set default priority
    message.timestamp = millis();
    message.length = packet.length;

    // Allocate memory for data
    message.data = new uint8_t[packet.length];

    // Copy payload
    for (uint16_t i = 0; i < packet.length; i++) {
        message.data[i] = packet.payload[i];
    }

    message.acknowledged = false;

    return message;
}

void LoRaSystem::retransmitPendingPackets() {
    // Get current time
    unsigned long now = millis();

    // Check each pending packet for retransmission
    for (auto it = sentPackets.begin(); it != sentPackets.end();) {
        // TODO: Add logic for when to retransmit (would need to add timestamp to LoRaPacket)
        // For now, we'll just keep them in the list and not retransmit
        ++it;
    }
}