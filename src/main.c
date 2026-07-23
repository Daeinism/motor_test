/*|LP-SCARA main|------------------------------------------------------------
#
# Project: Summer Project 2026
# Program: scara_main.c
#
# Description:
#  
#
# Author: Dain Kim
# Date Created: 2026-07-15
# Last Modified: 2026-07-17
# -----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdint.h> // For: Encoder
#include <stdbool.h> // For: PID
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h" //this is for PWM (NOT necessarily for LED)


/*|Macro|--------------------------------------------------------------------*/
#define TEST_LED_GPIO GPIO_NUM_36
#define LINK1_LEFT_LIMIT_GPIO GPIO_NUM_41
#define LINK1_RIGHT_LIMIT_GPIO GPIO_NUM_42
#define ENCODER_A_GPIO GPIO_NUM_17
#define ENCODER_B_GPIO GPIO_NUM_18

#define MOTOR1_IN1 GPIO_NUM_11
#define MOTOR1_IN2 GPIO_NUM_12
#define MOTOR2_IN1 GPIO_NUM_9
#define MOTOR2_IN2 GPIO_NUM_10

#define MOTOR_MAX_DUTY 1023
#define ENCODER_COUNTS_PER_REVOLUTION 1320 //Full Quadrature  Reading
#define POSITION_KP 1.0f // PID-P: Proportional Gain per error
#define POSITION_KD 0.3f // PID-D: Derivative
#define POSITION_MIN_DUTY 450 // recommended minimum (lower than 450 may result in weak output)
#define POSITION_MAX_DUTY 700 // 
#define POSITION_TOLERANCE 3 // Ex) Tolerance 3 × 360 / 1320 ≈ ±0.82° permitted


/*|Function Prototype|-------------------------------------------------------*/
static void gpioInit(void);
static void motorPwmInit(void);
static void ledTask(void *arg);
static void motorTask(void *arg);
static void userInputTask(void *arg);
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty);
static void limitSwitchTask(void *arg);
static void encoderISR(void *arg);
static void encoderInit(void);
static void encoderTask(void *arg);
static float getCurrentAngle(void);

/*|Variable Declaration|-----------------------------------------------------*/
    // static = makes the variable private for the lifetime of the program
    // volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static volatile int motorDuty = 0; 
static volatile int32_t encoderCount = 0;
static volatile int32_t targetEncoderCount = 0;
static volatile bool positionControlEnabled = true; // for lock or release
static volatile bool limitSwitchPressed = false;
static volatile uint8_t previousEncoderState = 0;
static const int8_t DRAM_ATTR encoderTransitionTable[16] = { //DRAM for variables/arrays
     0, -1,  1,  0,  // transition 0~3
     1,  0,  0, -1,  // transition 4~7
    -1,  0,  0,  1,  // transition 8~11
     0,  1, -1,  0   // transition 12~15
};

/*|Newly Added|---------------------------------------------------------------*/



/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    gpioInit();
    motorPwmInit();
    encoderInit();

    xTaskCreate(ledTask, "ledTask", 2048, NULL, 1, NULL);
    xTaskCreate(motorTask, "motorTask", 2048, NULL, 1, NULL);
    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
    xTaskCreate(limitSwitchTask, "limitSwitchTask", 2048, NULL, 5, NULL);
    xTaskCreate(encoderTask, "encoderTask", 4096, NULL, 1, NULL);
}

/*|Function Definition|------------------------------------------------------*/
static void gpioInit(void) // Initializing simple gpios
{
    gpio_reset_pin(TEST_LED_GPIO);
    gpio_set_direction(TEST_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TEST_LED_GPIO, 0);
}
static void motorPwmInit(void) // Setting up timer & channels
{

    // 0. Setting the timer for Pwm
    ledc_timer_config_t timer = { //configuration setting (won't need timer for anything else)
        .speed_mode = LEDC_LOW_SPEED_MODE, //default for ESP32 S3 Hardware (no need change) 
        .timer_num = LEDC_TIMER_0, // ESP32 S3 has 4 LEDC hardware timer & 8 Channels
        .duty_resolution = LEDC_TIMER_10_BIT, //get up to 1024 possible duty value
        .freq_hz = 20000, // common choice for DC motor
        .clk_cfg = LEDC_AUTO_CLK // (default) the driver picks clock source automatically 
    };
    ledc_timer_config(&timer); // pass the data to the function that programs the hardware

    // 1. Configuring each channel
    ledc_channel_config_t channels[] = {
        {
            .gpio_num = MOTOR1_IN1,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR1_IN2,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR2_IN1,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_2,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR2_IN2,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_3,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        }
    };

    for (int i = 0; i < 4; i++) { //applying the above configuration for all 4 channels
        ledc_channel_config(&channels[i]); 
    }
}
static void ledTask(void *arg) // Simple LED task for test
{
    while (1) 
    {
        gpio_set_level(TEST_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(TEST_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void motorTask(void *arg) // Processing Target & Error and tossing RequestedDuty to SetMotorDuty
{
    (void)arg; // telling compiler "Yes, we are not using the arguments. Stop asking."

    int previousDuty = 1;
    int32_t previousCountForDerivative = encoderCount; // for PID_D calculation

    while (1) {
        // 1. Setting up the variables 
        int32_t currentCount = encoderCount; // Snapshot the target value from encoderISR
        int32_t targetCount = targetEncoderCount; // Snapshot the target value from userInputTask
        int32_t positionError = targetCount - currentCount; // Get the positionError for later calculation
        float encoderVelocity = (currentCount - previousCountForDerivative) / 0.02f; //0.02 = 20ms
        int requestedDuty = 0; // Initializing the request value to 0 first.
        bool controlEnabled = positionControlEnabled && !limitSwitchPressed; // Updated by userInputTask

        previousCountForDerivative = currentCount;

        // 2. Determining the move direction
        if (controlEnabled && // only if the motor control is enabled
            (positionError > POSITION_TOLERANCE || positionError < -POSITION_TOLERANCE)) 
        {
            // P D calculations
            float controlOutput = (POSITION_KP * positionError) - (POSITION_KD * encoderVelocity);

            bool outputMovesTowardTarget = // Deciding whether or not need to move the motor
                (positionError > 0 && controlOutput > 0.0f) ||
                (positionError < 0 && controlOutput < 0.0f);

            if (outputMovesTowardTarget) {
                int dutyMagnitude;

                // Turning float into int value with direction
                if (controlOutput > 0.0f) {
                    dutyMagnitude = (int)controlOutput;
                } else {
                    dutyMagnitude = (int)(-controlOutput);
                }

                // Adjusting the value within the MIN & MAX duty value range
                if (dutyMagnitude < POSITION_MIN_DUTY) { 
                    dutyMagnitude = POSITION_MIN_DUTY;
                } else if (dutyMagnitude > POSITION_MAX_DUTY) {
                    dutyMagnitude = POSITION_MAX_DUTY;
                }

                // applying the direction of requestedDuty based on position Error
                if (controlOutput > 0.0f) {
                    requestedDuty = dutyMagnitude;
                } else {
                    requestedDuty = -dutyMagnitude;
                }
            }
        }

        motorDuty = requestedDuty;

        if (requestedDuty != previousDuty) {

            bool directionChanged = (requestedDuty > 0 && previousDuty < 0) || (requestedDuty < 0 && previousDuty > 0);
                // if either condition is met, then it means the direction has changed

            if (directionChanged) { // If direction is changed, Stop briefly before changing speed or direction. 
                setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, 0);
                setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // And then start moving again with new duty
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, requestedDuty);
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, requestedDuty);
            previousDuty = requestedDuty;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Refresh this duty cycle every 20ms
    }
}
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty)
{
    // 0. Declaring duty values (initially 0)
    uint32_t in1Duty = 0; //unsigned int 32bit type (required by LEDC)
    uint32_t in2Duty = 0;

    // 1. Capping to max duty (1023)
    if (signedDuty > MOTOR_MAX_DUTY) {
        signedDuty = MOTOR_MAX_DUTY;
    } 
    else if (signedDuty < -MOTOR_MAX_DUTY) {
        signedDuty = -MOTOR_MAX_DUTY;
    }

    // 2. Determining the direction
    if (signedDuty > 0) {
        in1Duty = (uint32_t)signedDuty;
    } 
    else if (signedDuty < 0) {
        in2Duty = (uint32_t)(-signedDuty);
    }

    // 3. Updating the duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, in1Channel, in1Duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, in1Channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, in2Channel, in2Duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, in2Channel);
}
static void userInputTask(void *arg) // Create targetEncoderCount from user angle input
{
    (void)arg; // Telling compiler "Don't need argument for this particular task, so don't ask"

    char inputBuffer[32];
    float inputDegrees;

    printf("Enter target angle from home in degrees, or type home/release/hold:\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20)); // loop every 20ms
            continue;
        }

        /*------------------------|Simple Homing Command|--------------------------*/
        if (strncmp(inputBuffer, "home", 4) == 0) {
            encoderCount = 0;
            targetEncoderCount = 0;
            printf("Current position set as home: 0.00 degrees\n");
            continue;
        }

        /*------------------------|Simple Release Command|--------------------------*/
        if (strncmp(inputBuffer, "release", 7) == 0) {
            positionControlEnabled = false;
            targetEncoderCount = encoderCount;
            printf("Position control released\n");
            continue;
        }

        /*------------------------|Simple Hold Command|-----------------------------*/
        if (strncmp(inputBuffer, "hold", 4) == 0) {
            if (limitSwitchPressed) {
                printf("Cannot hold while a limit switch is pressed\n");
                continue;
            }

            targetEncoderCount = encoderCount;
            positionControlEnabled = true;
            printf("Current position hold enabled\n");
            continue;
        }

        if (!positionControlEnabled) {
            printf("Position control is released. Type hold first.\n");
            continue;
        }

        inputDegrees = strtof(inputBuffer, NULL); // save degrees in float
 
        /*-------------------------|Angle to Target Count|----------------------------*/
        // Getting the targetCounts(float) value from the input degree
        float targetCountsFloat = inputDegrees * ENCODER_COUNTS_PER_REVOLUTION / 360.0f;
         
        // Round the float value to the nearest integer.
        int32_t targetCounts;

        // adding 0.5 or -0.5 before truncating into int for rounding
        if (targetCountsFloat >= 0.0f) { 
            targetCounts = (int32_t)(targetCountsFloat + 0.5f);
        } 
        else {
            targetCounts = (int32_t)(targetCountsFloat - 0.5f);
        }

        // Setting the targetcount based on home
        targetEncoderCount = targetCounts;

        printf("Target angle: %.2f degrees (%ld counts)\n",
               inputDegrees,
               (long)targetEncoderCount);
    }
}
static void limitSwitchTask(void *arg) 
{
    // 0. setting up the gpio configuation
    gpio_config_t switchConfig = {
        .pin_bit_mask =
            (1ULL << LINK1_LEFT_LIMIT_GPIO) |
            (1ULL << LINK1_RIGHT_LIMIT_GPIO),
            // << means, moving that 1(ON) sign to the left multiple times (# of gpio number)
            // bit mask is used because same config can also be applied to multiple gpio if wanted
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // setting interrupt function OFF
    };

    gpio_config(&switchConfig); // applying the above gpio configuration

    // 1. Detecting the change in limit switch state (1 = Unpressed as default state)
    int previousLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO); //get_level to get the value
    int previousRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

    while (1) {
        int currentLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
        int currentRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

        limitSwitchPressed = (currentLeftState == 0) || (currentRightState == 0);
            // 0 = button is pressed

        if (limitSwitchPressed && positionControlEnabled) {
            positionControlEnabled = false;
            targetEncoderCount = encoderCount;
            motorDuty = 0;
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, 0);
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, 0);
        }

        if (currentLeftState != previousLeftState) {
            if (currentLeftState == 0) {
                printf("Link 1 Left limit switch PRESSED\n");
            } else {
                printf("Link 1 Left limit switch RELEASED\n");
            }

            previousLeftState = currentLeftState;
        }

        if (currentRightState != previousRightState) {
            if (currentRightState == 0) {
                printf("Link 1 Right limit switch PRESSED\n");
            } else {
                printf("Link 1 Right limit switch RELEASED\n");
            }

            previousRightState = currentRightState;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
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

    // 2. Combining current & previous to make one 4-digit format (0~15 available)
    uint8_t transition = (previousEncoderState << 2) | currentState;

    // 3. Add 1, 0 ,-1 depending on the transition status according to the Table
    encoderCount += encoderTransitionTable[transition];

    // 4. Updating previous value
    previousEncoderState = currentState;

}
static void encoderInit(void) // Create encoder interrupt service
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
static float getCurrentAngle(void)
{
    int32_t currentCount = encoderCount;
    return ((float)currentCount * 360.0f) / ENCODER_COUNTS_PER_REVOLUTION;
}
static void encoderTask(void *arg) // Prints encoder value & Angle
{
    int32_t previousCount = encoderCount;

    while (1) {
        int32_t currentCount = encoderCount;

        if (currentCount != previousCount) {
            printf("Encoder count: %ld, Angle: %.2f degrees\n", (long)currentCount, getCurrentAngle());
            previousCount = currentCount;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
