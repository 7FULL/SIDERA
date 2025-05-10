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
#include <Arduino.h>
#include "../CommunicationSystem.h"
#include "../StorageSystems/StorageManager.h"

class LoRaSystem : public CommunicationSystem {
public:
    LoRaSystem(SPIClass& spi, int8_t csPin, int8_t resetPin, int8_t irqPin, StorageManager* storageManager = nullptr);
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
    bool setSpreadingFactor(int sf);
    bool setSignalBandwidth(long bandwidth);
    bool setCodingRate(int denominator);
    bool enableCrc();
    bool disableCrc();

private:
    SPIClass& spi;
    int8_t csPin;
    int8_t resetPin;
    int8_t irqPin;
    StorageManager* storageManager;

    bool receiving = false; // Flag to indicate if the sensor is in receive mode

    uint8_t nodeId = 1;           // Default node ID
    uint8_t destinationId = 0;    // Default destination (0 = broadcast)
    uint8_t packetCounter = 0;    // Packet counter

    int lastRssi = 0;             // RSSI of last packet
    float lastSnr = 0.0f;         // SNR of last packet
    bool lowPowerMode = false;    // Whether low power mode is enabled

    std::queue<Message> receivedMessages; // Queue of received messages

    // Process incoming packet
    void processPacket(int packetSize);

    // Static callback for received packets
    static void onReceiveStatic(int packetSize);
    static LoRaSystem* instance;
};

#endif // LORA_SYSTEM_H