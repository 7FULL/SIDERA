/**
 * LoRa Communication System Implementation
 *
 * Implementation for SX1276/77/78/79 LoRa modules
 */

#ifndef LORA_SYSTEM_H
#define LORA_SYSTEM_H

#include <SPI.h>
#include <LoRa.h>
#include <queue>
#include <vector>
#include <Arduino.h>
#include "../CommunicationSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"

// LoRa packet format
struct LoRaPacket {
    uint8_t destination;   // Destination node ID
    uint8_t source;        // Source node ID
    uint8_t id;            // Packet ID
    uint8_t flags;         // Control flags (ack request, etc.)
    MessageType type;      // Message type
    uint16_t length;       // Payload length
    std::vector<uint8_t> payload; // Message payload
    unsigned long sentTime;      // Time when packet was sent
    uint8_t retryCount;          // Number of retransmission attempts

    // Control flag bits
    static const uint8_t FLAG_ACK_REQUEST = 0x01;
    static const uint8_t FLAG_ACK = 0x02;
    static const uint8_t FLAG_RETRANSMISSION = 0x04;
};

class LoRaSystem : public CommunicationSystem {
public:
    LoRaSystem(SPIClass& spi, int8_t csPin, int8_t resetPin, int8_t irqPin, int8_t txEnPin = -1, int8_t rxEnPin = -1, StorageManager* storageManager = nullptr);
    ~LoRaSystem() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement CommunicationSystem interface
    bool sendMessage(const Message& message) override;
    bool hasReceivedMessages() override;
    bool getNextMessage(Message& message) override;
    void setNodeId(uint8_t id) override;
    uint8_t getNodeId() const override;
    void setDestinationId(uint8_t id) override;
    int getLastRssi() override;
    float getLastSnr() override;
    bool setTxPower(int8_t power) override;
    bool enableLowPowerMode() override;
    bool disableLowPowerMode() override;

    // LoRa specific methods
    bool setFrequency(long frequency);
    bool setSignalBandwidth(long bandwidth);
    bool setSpreadingFactor(int sf);
    bool setCodingRate(int denominator);
    bool enableCrc();
    bool disableCrc();

private:
    SPIClass& spi;
    int8_t csPin;
    int8_t resetPin;
    int8_t irqPin;
    int8_t txEnPin;
    int8_t rxEnPin;
    StorageManager* storageManager;

    uint8_t nodeId = 1;           // Default node ID
    uint8_t destinationId = 0;    // Default destination (0 = broadcast)
    uint8_t nextPacketId = 0;     // Next packet ID to use

    int lastRssi = 0;             // RSSI of last packet
    float lastSnr = 0.0f;         // SNR of last packet
    bool lowPowerMode = false;    // Whether low power mode is enabled

    std::queue<Message> receivedMessages; // Queue of received messages
    std::vector<LoRaPacket> sentPackets;  // History of sent packets for retransmission

    // Process incoming packet
    void processPacket(int packetSize);

    // Send a LoRa packet
    bool sendPacket(const LoRaPacket& packet);

    // Handle acknowledgements
    void handleAcknowledgement(const LoRaPacket& packet);

    // Convert Message to LoRaPacket
    LoRaPacket messageToPacket(const Message& message);

    // Convert LoRaPacket to Message
    Message packetToMessage(const LoRaPacket& packet);

    // Retransmit unacknowledged packets
    void retransmitPendingPackets();

    static LoRaSystem* instance;
    static void onReceiveStatic(int packetSize);
};

#endif // LORA_SYSTEM_H