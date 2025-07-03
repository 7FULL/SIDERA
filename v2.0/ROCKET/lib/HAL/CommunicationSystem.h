#ifndef COMMUNICATION_SYSTEM_H
#define COMMUNICATION_SYSTEM_H

#include "Sensor.h" // Reusamos la interfaz Sensor para mantener coherencia

enum class MessageType {
    TELEMETRY,         // Regular telemetry data
    EVENT,             // Event notification (launch, apogee, etc.)
    COMMAND_RESPONSE,  // Response to a command
    STATUS,            // System status update
    LOG,               // Log message
    ALERT             // Critical alert
};

struct Message {
    MessageType type;
    uint8_t priority;     // 0-255, higher number = higher priority
    uint32_t timestamp;   // Timestamp in milliseconds
    uint8_t* data;        // Pointer to message data
    uint16_t length;      // Length of data in bytes
    bool acknowledged;    // Whether the message has been acknowledged
};

class CommunicationSystem : public Sensor {
public:
    virtual ~CommunicationSystem() = default;

    // Send a message
    virtual bool sendMessage(const Message& message) = 0;

    // Check if there are any received messages
    virtual bool hasReceivedMessages() = 0;

    // Get the next received message (returns false if no messages)
    virtual bool getNextMessage(Message& message) = 0;

    // Set the node identifier
    virtual void setNodeId(uint8_t id) = 0;

    // Get the node identifier
    virtual uint8_t getNodeId() const = 0;

    // Set the destination node for transmissions
    virtual void setDestinationId(uint8_t id) = 0;

    // Get the RSSI (Received Signal Strength Indicator) of the last received packet
    virtual int getLastRssi() = 0;

    // Get the SNR (Signal-to-Noise Ratio) of the last received packet
    virtual float getLastSnr() = 0;

    // Set transmit power (if supported)
    virtual bool setTxPower(int8_t power) = 0;

    // Enter low power mode (if supported)
    virtual bool enableLowPowerMode() = 0;

    // Exit low power mode (if supported)
    virtual bool disableLowPowerMode() = 0;
};

#endif