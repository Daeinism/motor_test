#include "encoder.h"

#include <stdint.h> // For: Encoder

#include "freertos/FreeRTOS.h" // do not remove this

#include "driver/gpio.h"

#define ENCODER_A_GPIO GPIO_NUM_39
#define ENCODER_B_GPIO GPIO_NUM_38

static void encoderISR(void *arg);

// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static volatile int32_t encoderCount = 0;
static volatile uint8_t previousEncoderState = 0;
static const int8_t DRAM_ATTR encoderTransitionTable[16] = { //DRAM for variables/arrays
     0, -1,  1,  0,  // transition 0~3
     1,  0,  0, -1,  // transition 4~7
    -1,  0,  0,  1,  // transition 8~11
     0,  1, -1,  0   // transition 12~15
};

void encoderInit(void) // Create encoder interrupt service
{
    // 0. Setting up the GPIO pin config for encoder wires
    gpio_config_t encoderConfig = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO),
            // 1ULL = 1 Unsigned Long Long (64bit)
            // If gpio is 9, then "Move 1 to the left 9 times" -> 000100000000
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&encoderConfig); //apply the above setup

    previousEncoderState = (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);

    /* 1. Setting the interruption condition. Options:
        POSEDGE: LOW → HIGH     
        NEGEDGE: HIGH → LOW    
        ANYEDGE: ANY
        LOW_LEVEL: while LOW
        HIGH_LEVEL: while HIGH                        */
    gpio_set_intr_type(ENCODER_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_B_GPIO, GPIO_INTR_ANYEDGE);
    
    /* 2. Installing a service that can handle above gpio interrupt
        - installing just once is sufficient for the entire program (ESP32 firmware)
        - the public ISR service is now saved in GPIO driver internally */  
    gpio_install_isr_service(0); // 0 = default setting

    // 3. Registering encoderISR to selected GPIO pins
    gpio_isr_handler_add(ENCODER_A_GPIO, encoderISR, NULL);
    gpio_isr_handler_add(ENCODER_B_GPIO, encoderISR, NULL);
}

int32_t encoderGetCount(void) // used by main to send the encoder value to the motorTask for PID calculation
{
    return encoderCount; // Sharing the private variable with other files (like main.c) through this function
}

void encoderResetCount(void) // used by main.userInputTask for homing
{
    encoderCount = 0;
}

static void IRAM_ATTR encoderISR(void *arg) // Determine direction & Update encoder value
{ 
    // IRAM_ATTR: "Put this function inside the IRAM"
    // ISR: "Interrupt Service Routine"

    // 1. Putting A/B pin readings into one 2-digit format
    uint8_t currentState = (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);
        // reading bitwise, A is at 2nd digit, B is at 1st digit (from the right)
        // if A=1, B=1, then it reads 11
        // | sign is for combining two digits.

    // 2. Combining current & previous to make one 4-digit format (0~15 available) like 0101
    uint8_t transition = (previousEncoderState << 2) | currentState;

    // 3. Add 1, 0 ,-1 depending on the transition status according to the Table
    encoderCount += encoderTransitionTable[transition];

    // 4. Updating previous value
    previousEncoderState = currentState;

}

