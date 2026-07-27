#ifndef CAN_HELPER_H
#define CAN_HELPER_H

#include <Arduino.h>
#include <driver/twai.h>

class CANHelper {
public:

    /**
     * @brief Initialize CAN bus with specified pins
     * 
     * @param txPin GPIO pin for CAN TX
     * @param rxPin GPIO pin for CAN RX
     * @return true if initialization successful
     */
    static bool setupCAN(gpio_num_t rxPin, gpio_num_t txPin);

    /**
     * @brief Initialize CAN bus with specified pins but filters out only 0x10, 0x11, 0x20 and 0x21
     * 
     * @param txPin GPIO pin for CAN TX
     * @param rxPin GPIO pin for CAN RX
     * @return true if initialization successful
     */
    static bool setupFilterCAN(gpio_num_t rxPin, gpio_num_t txPin);

    /**
     * @brief Send a CAN message and print debug info to Serial
     * 
     * @param message The TWAI/CAN message to transmit
     * @param debugPrint Enable/disable Serial debug output (default: true)
     * @return true if transmission successful, false otherwise
     */
    static bool sendMessage(twai_message_t& message, bool debugPrint = true);
    
    /**
     * @brief Send a CAN message without debug output (silent mode)
     */
    static bool sendMessageSilent(twai_message_t& message);
    
    /**
     * @brief Format a CAN message as a string (useful for logging)
     * 
     * @param message The message to format
     * @param buffer Output buffer (must be at least 64 bytes)
     * @param timestamp Optional timestamp to include
     */
    static void formatMessage(twai_message_t& message, char* buffer, unsigned long timestamp = 0);

    /**
     * @brief Prints the can message to the serial output
     * 
     * @param message The message to print
     * @param messageCount The number of messages received
     */
    static void printCANMessage(twai_message_t &message, int messageCount);

private:
    static void printHexByte(byte b);
    static void printHexWithPadding(uint32_t value, int minWidth);
};

#endif