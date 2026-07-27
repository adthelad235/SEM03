#include <stdint.h>
#include <stdbool.h>

class AcceleratorPedal {
 
private:
    static uint8_t errorCode;
    static uint16_t lastPot1Value;
    static uint16_t lastPot2Value;
    static uint8_t samplesSincePot1Change;
    static uint8_t samplesSincePot2Change;

public:          

static uint16_t potentiometerReading(uint16_t value1, uint16_t value2) {
    if (value1 == lastPot1Value && value1 != 0 && value1 != 4095){
        samplesSincePot1Change++;
    }
    else{
        lastPot1Value = value1;
        samplesSincePot1Change = 0;
    }
    if (value2 == lastPot2Value  && value2 != 0 && value2 != 4095){
        samplesSincePot2Change++;
    }
    else{
        lastPot2Value = value2;
        samplesSincePot2Change = 0;
    }
    
    // Check for disconnected/shorted potentiometers
    bool pot1Error = (value1 > POT_DISCONNECTED_THRESHOLD || value1 < POT_MIN_VALID);
    bool pot2Error = (value2 > POT_DISCONNECTED_THRESHOLD || value2 < POT_MIN_VALID);
    
    if (pot1Error && pot2Error){
        errorCode = 2;  // Both pots failed
        return (uint16_t)0xFFFF;
    }
    if (pot1Error){
        errorCode = 3;  // Pot 1 failed
        return value2;
    }
    if (pot2Error){
        errorCode = 4;  // Pot 2 failed
        return value1;
    }
        if (value1 == 0 && value2 > 500){
        errorCode = 8;
        return value2;
    }
    if (value2 == 0 && value1 > 500){
        errorCode = 9;
        return value1;
    }
    if (value1 == 4095 && value2 < 3500){
        errorCode = 10;
        return value2;
    }
    if (value2 == 4095 && value1 < 3500){
        errorCode = 11;
        return value1;
    }
    if (samplesSincePot1Change > 50 && samplesSincePot2Change > 50){
        errorCode = 5;
        return (uint16_t)0xFFFF;
    }
    if (samplesSincePot1Change > 50){
        errorCode = 6;
        return value2;
    }
    if (samplesSincePot2Change > 50){
        errorCode = 7;
        return value1;
    }


    uint32_t sum = (uint32_t)value1 + (uint32_t)value2;
    uint16_t average = (uint16_t)(sum / 2);
    
    return average;
}

    static uint8_t gearPosition(bool value1, bool value2){
        if (value1 && value2){
            errorCode = 1;
            return (uint8_t) 3;
        }
        if (value1){
            return (uint8_t) 1;
        }
        if (value2){
            return (uint8_t) 2;
        }
        return (uint8_t) 0;
    }

    static uint8_t getError(){
        return errorCode;
    }

    static void resetError(){
        errorCode = 0;
    }
};

uint8_t AcceleratorPedal::errorCode = 0;
uint16_t AcceleratorPedal::lastPot1Value = 0;
uint16_t AcceleratorPedal::lastPot2Value = 0;
uint8_t AcceleratorPedal::samplesSincePot1Change = 0;
uint8_t AcceleratorPedal::samplesSincePot2Change = 0;