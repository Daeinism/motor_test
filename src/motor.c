#include "motor.h"

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h" // this is for PWM (NOT necessarily for LED)

#include "pidCalculator.h"

#define MOTOR1_IN1 GPIO_NUM_1
#define MOTOR1_IN2 GPIO_NUM_2
#define MOTOR2_IN1 GPIO_NUM_41
#define MOTOR2_IN2 GPIO_NUM_42

#define MOTOR_MAX_DUTY 1023


// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static void motorPwmInit(void);
static void motorTask(void *arg);
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty);
static void setAllMotorDuty(int signedDuty);

static volatile int32_t link1TargetEncoderCount = 0;
static volatile int32_t link2TargetEncoderCount = 0;
static volatile bool positionControlEnabled = true; // for lock or release
static MotorEncoderCountReader readLink1EncoderCount = NULL;
static MotorEncoderCountReader readLink2EncoderCount = NULL;
static PidCalculatorState link1PidState = {0};
static PidCalculatorState link2PidState = {0};

void motorInit(MotorEncoderCountReader link1EncoderCountReader, MotorEncoderCountReader link2EncoderCountReader)
{
    readLink1EncoderCount = link1EncoderCountReader;
    readLink2EncoderCount = link2EncoderCountReader;
    pidCalculatorReset(&link1PidState);
    pidCalculatorReset(&link2PidState);
    motorPwmInit();
    xTaskCreate(motorTask, "motorTask", 2048, NULL, 1, NULL);
}

void motorSetLink1TargetCount(int32_t targetCount)
{
    link1TargetEncoderCount = targetCount; // targetCount comes from main.userInputTask (user input degrees → targetCounts)
}

void motorSetLink2TargetCount(int32_t targetCount)
{
    link2TargetEncoderCount = targetCount;
}

void motorHold(void) // used by main.userInputTask
{
    if (readLink1EncoderCount != NULL) {
        link1TargetEncoderCount = readLink1EncoderCount();
    }
    if (readLink2EncoderCount != NULL) {
        link2TargetEncoderCount = readLink2EncoderCount();
    }

    positionControlEnabled = true;
}

void motorRelease(void) // used by main.userInputTask
{
    positionControlEnabled = false;

    if (readLink1EncoderCount != NULL) {
        link1TargetEncoderCount = readLink1EncoderCount(); // set current position as target position when releasing the motor
    }
    if (readLink2EncoderCount != NULL) {
        link2TargetEncoderCount = readLink2EncoderCount();
    }
}

void motorEmergencyStop(void) // registered as the limit switch pressed handler
{
    positionControlEnabled = false;

    if (readLink1EncoderCount != NULL) {
        link1TargetEncoderCount = readLink1EncoderCount();
    }
    if (readLink2EncoderCount != NULL) {
        link2TargetEncoderCount = readLink2EncoderCount();
    }

    setAllMotorDuty(0);
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

    int previousLink1Duty = 1;
    int previousLink2Duty = 1;

    while (1) {
        // 1. Setting up the variables 
        int32_t currentLink1Count = readLink1EncoderCount(); // Snapshot the target value from encoderISR
        int32_t currentLink2Count = readLink2EncoderCount();
        int32_t link1TargetCount = link1TargetEncoderCount; // Snapshot the target value from userInputTask
        int32_t link2TargetCount = link2TargetEncoderCount;
        int requestedLink1Duty = 0; // Initializing the request value to 0 first.
        int requestedLink2Duty = 0;
        bool controlEnabled = positionControlEnabled; // Updated by userInputTask

        if (controlEnabled) {
            requestedLink1Duty = pidCalculatorUpdate(
                &link1PidState,
                link1TargetCount,
                currentLink1Count,
                0.02f
            );
            requestedLink2Duty = pidCalculatorUpdate(
                &link2PidState,
                link2TargetCount,
                currentLink2Count,
                0.02f
            );
                // 0.02f = 20ms, the time interval between each motorTask loop
        } else {
            pidCalculatorReset(&link1PidState);
            pidCalculatorReset(&link2PidState);
        }

        bool link1DirectionChanged =
            (requestedLink1Duty > 0 && previousLink1Duty < 0) ||
            (requestedLink1Duty < 0 && previousLink1Duty > 0);
        bool link2DirectionChanged =
            (requestedLink2Duty > 0 && previousLink2Duty < 0) ||
            (requestedLink2Duty < 0 && previousLink2Duty > 0);

        // Brief stopping mechanism when changing direction to prevent overshoot and oscillation
        if (requestedLink1Duty != previousLink1Duty && link1DirectionChanged) {
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, 0);
        }
        if (requestedLink2Duty != previousLink2Duty && link2DirectionChanged) {
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, 0);
        }

        if (link1DirectionChanged || link2DirectionChanged) {
                // if either condition is met, then it means the direction has changed
            vTaskDelay(pdMS_TO_TICKS(10)); // If direction is changed, Stop briefly before changing speed or direction.
        }

        // And then start moving again with new duty
        if (requestedLink1Duty != previousLink1Duty) {
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, requestedLink1Duty);
            previousLink1Duty = requestedLink1Duty;
        }
        if (requestedLink2Duty != previousLink2Duty) {
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, requestedLink2Duty);
            previousLink2Duty = requestedLink2Duty;
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
