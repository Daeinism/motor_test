#include "motor.h"

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h" //this is for PWM (NOT necessarily for LED)

#include "pidCalculator.h"

#define MOTOR1_IN1 GPIO_NUM_1
#define MOTOR1_IN2 GPIO_NUM_2
#define MOTOR2_IN1 GPIO_NUM_21
#define MOTOR2_IN2 GPIO_NUM_47

#define MOTOR_MAX_DUTY 1023


// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static void motorPwmInit(void);
static void motorTask(void *arg);
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty);
static void setAllMotorDuty(int signedDuty);

static volatile int32_t targetEncoderCount = 0;
static volatile bool positionControlEnabled = true; // for lock or release
static MotorEncoderCountReader readEncoderCount = NULL;

void motorInit(MotorEncoderCountReader encoderCountReader)
{
    readEncoderCount = encoderCountReader;
    pidCalculatorReset();
    motorPwmInit();
    xTaskCreate(motorTask, "motorTask", 2048, NULL, 1, NULL);
}

void motorSetTargetCount(int32_t targetCount)
{
    targetEncoderCount = targetCount; // targetCount comes from main.userInputTask (user input degrees → targetCounts)
}

void motorHold(void) // used by main.userInputTask
{
    if (readEncoderCount != NULL) {
        targetEncoderCount = readEncoderCount();
    }

    positionControlEnabled = true;
}

void motorRelease(void) // used by main.userInputTask
{
    positionControlEnabled = false;

    if (readEncoderCount != NULL) {
        targetEncoderCount = readEncoderCount(); // set current position as target position when releasing the motor
    }
}

void motorEmergencyStop(void) // used by main.limitSwitchTask
{
    positionControlEnabled = false;

    if (readEncoderCount != NULL) {
        targetEncoderCount = readEncoderCount();
    }

    setAllMotorDuty(0);
}

void IRAM_ATTR motorDisableControlFromISR(void) // used by main.limitSwitchISR
{
    positionControlEnabled = false;
}

bool motorIsControlEnabled(void)
{
    return positionControlEnabled; // true = motor is holding position, false = motor is released
    // the purpose of this function is to share a private variable (positionControlEnabled) with other files (main.c)
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

static void motorTask(void *arg) // Processing Target & Error and tossing RequestedDuty to SetMotorDuty
{
    (void)arg; // telling compiler "Yes, we are not using the arguments. Stop asking."

    int previousDuty = 1;

    while (1) {
        // 1. Setting up the variables 
        int32_t currentCount = readEncoderCount(); // Snapshot the target value from encoderISR
        int32_t targetCount = targetEncoderCount; // Snapshot the target value from userInputTask
        int requestedDuty = 0; // Initializing the request value to 0 first.
        bool controlEnabled = positionControlEnabled; // Updated by userInputTask

        if (controlEnabled) {
            requestedDuty = pidCalculatorUpdate(targetCount, currentCount, 0.02f);
                // 0.02f = 20ms, the time interval between each motorTask loop
        } else {
            pidCalculatorReset();
        }

        if (requestedDuty != previousDuty) {

            // Brief stopping mechanism when changing direction to prevent overshoot and oscillation
            bool directionChanged = (requestedDuty > 0 && previousDuty < 0) || (requestedDuty < 0 && previousDuty > 0);
                // if either condition is met, then it means the direction has changed

            if (directionChanged) { // If direction is changed, Stop briefly before changing speed or direction. 
                setAllMotorDuty(0);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // And then start moving again with new duty
            setAllMotorDuty(requestedDuty);
            previousDuty = requestedDuty;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Refresh this duty cycle every 20ms
    }
}

static void setAllMotorDuty(int signedDuty)
{
    setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, signedDuty);
    setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, signedDuty);
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
