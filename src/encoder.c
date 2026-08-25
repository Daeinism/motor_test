#include "encoder.h"

#include <stdint.h> // For: Encoder
#include <stdio.h>

#include "freertos/FreeRTOS.h" // do not remove this
#include "freertos/task.h"

#include "driver/gpio.h"

#define LINK1_ENCODER_A_GPIO GPIO_NUM_39
#define LINK1_ENCODER_B_GPIO GPIO_NUM_38
#define LINK2_ENCODER_A_GPIO GPIO_NUM_36
#define LINK2_ENCODER_B_GPIO GPIO_NUM_37

static void link1EncoderISR(void *arg);
static void link2EncoderISR(void *arg);
static void encoderPrintTask(void *arg);
static float getAngleFromCount(int32_t encoderCount);

// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static volatile int32_t link1EncoderCount = 0;
static volatile int32_t link2EncoderCount = 0;
static volatile uint8_t previousLink1EncoderState = 0;
static volatile uint8_t previousLink2EncoderState = 0;
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
        .pin_bit_mask =
            (1ULL << LINK1_ENCODER_A_GPIO) |
            (1ULL << LINK1_ENCODER_B_GPIO) |
            (1ULL << LINK2_ENCODER_A_GPIO) |
            (1ULL << LINK2_ENCODER_B_GPIO),
            // 1ULL = 1 Unsigned Long Long (64bit)
            // If gpio is 9, then "Move 1 to the left 9 times" -> 000100000000
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&encoderConfig); //apply the above setup

    previousLink1EncoderState =
        (gpio_get_level(LINK1_ENCODER_A_GPIO) << 1) |
        gpio_get_level(LINK1_ENCODER_B_GPIO);
    previousLink2EncoderState =
        (gpio_get_level(LINK2_ENCODER_A_GPIO) << 1) |
        gpio_get_level(LINK2_ENCODER_B_GPIO);

    /* 1. Setting the interruption condition. Options:
        POSEDGE: LOW → HIGH     
        NEGEDGE: HIGH → LOW    
        ANYEDGE: ANY
        LOW_LEVEL: while LOW
        HIGH_LEVEL: while HIGH                        */
    gpio_set_intr_type(LINK1_ENCODER_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(LINK1_ENCODER_B_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(LINK2_ENCODER_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(LINK2_ENCODER_B_GPIO, GPIO_INTR_ANYEDGE);
    
    /* 2. Installing a service that can handle above gpio interrupt
        - installing just once is sufficient for the entire program (ESP32 firmware)
        - the public ISR service is now saved in GPIO driver internally */  
    gpio_install_isr_service(0); // 0 = default setting

    // 3. Registering encoderISR to selected GPIO pins
    gpio_isr_handler_add(LINK1_ENCODER_A_GPIO, link1EncoderISR, NULL);
    gpio_isr_handler_add(LINK1_ENCODER_B_GPIO, link1EncoderISR, NULL);
    gpio_isr_handler_add(LINK2_ENCODER_A_GPIO, link2EncoderISR, NULL);
    gpio_isr_handler_add(LINK2_ENCODER_B_GPIO, link2EncoderISR, NULL);

    xTaskCreate(encoderPrintTask, "encoderPrintTask", 4096, NULL, 1, NULL);
}

int32_t encoderGetLink1Count(void) // used by main to send the encoder value to the motorTask for PID calculation
{
    return link1EncoderCount; // Sharing the private variable with other files (like main.c) through this function
}
int32_t encoderGetLink2Count(void)
{
    return link2EncoderCount;
}

void encoderResetLink1Count(void) // used by main.userInputTask for homing
{
    link1EncoderCount = 0;
}
void encoderResetLink2Count(void)
{
    link2EncoderCount = 0;
}

static void IRAM_ATTR link1EncoderISR(void *arg) // Determine direction & Update encoder value
{ 
    // IRAM_ATTR: "Put this function inside the IRAM"
    // ISR: "Interrupt Service Routine"

    // 1. Putting A/B pin readings into one 2-digit format
    uint8_t currentState =
        (gpio_get_level(LINK1_ENCODER_A_GPIO) << 1) |
        gpio_get_level(LINK1_ENCODER_B_GPIO);
        // reading bitwise, A is at 2nd digit, B is at 1st digit (from the right)
        // if A=1, B=1, then it reads 11
        // | sign is for combining two digits.

    // 2. Combining current & previous to make one 4-digit format (0~15 available) like 0101
    uint8_t transition = (previousLink1EncoderState << 2) | currentState;

    // 3. Add 1, 0 ,-1 depending on the transition status according to the Table
    link1EncoderCount += encoderTransitionTable[transition];

    // 4. Updating previous value
    previousLink1EncoderState = currentState;

}
static void IRAM_ATTR link2EncoderISR(void *arg)
{
    uint8_t currentState =
        (gpio_get_level(LINK2_ENCODER_A_GPIO) << 1) |
        gpio_get_level(LINK2_ENCODER_B_GPIO);

    uint8_t transition = (previousLink2EncoderState << 2) | currentState;

    link2EncoderCount += encoderTransitionTable[transition];

    previousLink2EncoderState = currentState;
}

static float getAngleFromCount(int32_t encoderCount)
{
    return ((float)encoderCount * 360.0f) / ENCODER_COUNTS_PER_REVOLUTION;
}
static void encoderPrintTask(void *arg) // Prints encoder value & Angle
{
    (void)arg;

    int32_t previousLink1Count = encoderGetLink1Count();
    int32_t previousLink2Count = encoderGetLink2Count();

    while (1) {
        int32_t currentLink1Count = encoderGetLink1Count();
        int32_t currentLink2Count = encoderGetLink2Count();

        if ((currentLink1Count != previousLink1Count) ||
            (currentLink2Count != previousLink2Count)) {
            printf("Link 1: %ld counts, %.2f degrees | Link 2: %ld counts, %.2f degrees\n",
                   (long)currentLink1Count,
                   getAngleFromCount(currentLink1Count),
                   (long)currentLink2Count,
                   getAngleFromCount(currentLink2Count));
            previousLink1Count = currentLink1Count;
            previousLink2Count = currentLink2Count;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
