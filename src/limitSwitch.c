#include "limitSwitch.h"

#include <stddef.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#define LINK1_LEFT_LIMIT_GPIO GPIO_NUM_4
#define LINK1_RIGHT_LIMIT_GPIO GPIO_NUM_6
#define LIMIT_SWITCH_DEBOUNCE_MS 25

static void limitSwitchTask(void *arg);
static void limitSwitchISR(void *arg);

static volatile bool leftSwitchPressed = false;
static volatile bool rightSwitchPressed = false;
static volatile bool anySwitchPressed = false;
static int previousLeftState = 1;
static int previousRightState = 1;
static TaskHandle_t limitSafetyTaskHandle = NULL; // Create an Empty Handle (works like a container)
static LimitSwitchPressedHandler onLimitSwitchPressed = NULL;

void limitSwitchInit(LimitSwitchPressedHandler pressedHandler)
{
    onLimitSwitchPressed = pressedHandler;
        // saving motor.emergencyStop function pointer to the private variable 

    // ------------------------- GPIO Configuration ----------------------------
    gpio_config_t switchConfig = {
        .pin_bit_mask =
            (1ULL << LINK1_LEFT_LIMIT_GPIO) |
            (1ULL << LINK1_RIGHT_LIMIT_GPIO),
            // << means, moving that 1(ON) sign to the left multiple times (# of gpio number)
            // bit mask is used because same config can also be applied to multiple gpio if wanted
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&switchConfig); // applying the above gpio configuration

    // ------------------------- Initial State Check ----------------------------
        
    previousLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO); //get_level to get the value
    previousRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

    // (1 = Unpressed as default state)
    // (0 = Pressed, because the limit switch is connected to GND when pressed)
    leftSwitchPressed = previousLeftState == 0; 
    rightSwitchPressed = previousRightState == 0;
    anySwitchPressed = leftSwitchPressed || rightSwitchPressed;
        // if either previous state is 0, then it means the limit switch is newly pressed

    xTaskCreate(limitSwitchTask, "limitSwitchTask", 4096, NULL, 10, &limitSafetyTaskHandle); 
        // Priority 10 (Higher than others)

    // ------------------------- Interrupt Configuration --------------------------
    gpio_isr_handler_add(LINK1_LEFT_LIMIT_GPIO, limitSwitchISR, NULL);
    gpio_isr_handler_add(LINK1_RIGHT_LIMIT_GPIO, limitSwitchISR, NULL);

    gpio_set_intr_type(LINK1_LEFT_LIMIT_GPIO, GPIO_INTR_ANYEDGE); // Any change will trigger the interrupt  
    gpio_set_intr_type(LINK1_RIGHT_LIMIT_GPIO, GPIO_INTR_ANYEDGE);

    leftSwitchPressed = gpio_get_level(LINK1_LEFT_LIMIT_GPIO) == 0;
    rightSwitchPressed = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO) == 0;
    anySwitchPressed = leftSwitchPressed || rightSwitchPressed;
 
    // ------------------------- Initial Notification --------------------------
    if (anySwitchPressed) {
        xTaskNotifyGive(limitSafetyTaskHandle);
        /*“If a limit switch is already pressed by the time it initiates, give the task 
        one notification so the first loop iteration checks and stops the motors.”*/
    } 
}

bool limitSwitchIsAnyPressed(void)
{
    return anySwitchPressed;
}
bool limitSwitchIsLeftPressed(void)
{
    return leftSwitchPressed;
}
bool limitSwitchIsRightPressed(void)
{
    return rightSwitchPressed;
}

static void limitSwitchTask(void *arg) 
{
    (void)arg;

    while (1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Run the rest of the loop ONLY IF the notify is taken.
        /* pd = Portable Definition
        pdTRUE: Clear all pending notifications after waking up.
        pdFALSE: Consume one notification at a time.
        portMAX_DELAY: Wait indefinitely until a notification is received.
        */

        while (1) {
            int currentLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            int currentRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

            //------------------------- Emergency Stop --------------------------------
            // 0 is pressed, 1 is unpressed for left and right limit switches (because they are connected to GND when pressed)
            // 0 is unpressed, 1 is pressed for anySwitchPressed (because it is a logical)
            if ((currentLeftState == 0) || (currentRightState == 0) || anySwitchPressed) {
                anySwitchPressed = true;

                if (onLimitSwitchPressed != NULL) {
                    // <<<<<<<<<<<<<<<<<<EMERGENCY STOP FUNCTION POINTER (motorEmergencyStop) IS CALLED HERE>>>>>>>>>>>>>>>>
                    onLimitSwitchPressed(); // == motor.motorEmergencyStop();  
                }
            }

            //------------------------ Print (PRESSED) ---------------------------
            if ((currentLeftState == 0) && (previousLeftState != 0)) {
                leftSwitchPressed = true;
                printf("Link 1 Left limit switch PRESSED\n");
                previousLeftState = 0;
            }

            if ((currentRightState == 0) && (previousRightState != 0)) {
                rightSwitchPressed = true;
                printf("Link 1 Right limit switch PRESSED\n");
                previousRightState = 0;
            }

            //------------------------- Debouncing ----------------------------
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LIMIT_SWITCH_DEBOUNCE_MS)) != 0) {
                continue;
            }

            currentLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            currentRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

            //------------------------ Print (RELEASED) ---------------------------
            if ((currentLeftState != previousLeftState) && (currentLeftState != 0)) {
                printf("Link 1 Left limit switch RELEASED\n");
                previousLeftState = currentLeftState;
            }

            if ((currentRightState != previousRightState) && (currentRightState != 0)) {
                printf("Link 1 Right limit switch RELEASED\n");
                previousRightState = currentRightState;
            }

            leftSwitchPressed = currentLeftState == 0;
            rightSwitchPressed = currentRightState == 0;
            anySwitchPressed = leftSwitchPressed || rightSwitchPressed;
                // 0 = button is pressed

            break;
        }
    }
}

static void IRAM_ATTR limitSwitchISR(void *arg)
{
    (void)arg;

    BaseType_t higherPriorityTaskWoken = pdFALSE; //pdFALSE = FALSE (no other meaning than FALSE)
        // "Initially, no higher-priority task has been woken"

    bool anyLimitSwitchPressed =
        (gpio_get_level(LINK1_LEFT_LIMIT_GPIO) == 0) ||
        (gpio_get_level(LINK1_RIGHT_LIMIT_GPIO) == 0);

    if (anyLimitSwitchPressed) {
        anySwitchPressed = true;
    }

    if (limitSafetyTaskHandle != NULL) { // If the handle has been assigned to a valid task
        vTaskNotifyGiveFromISR(
            limitSafetyTaskHandle, // "Give the notification to the task that is waiting for it"
            &higherPriorityTaskWoken /*"If the task that is waiting for the notification 
            has a higher priority than the currently running task, set this variable to pdTRUE"*/
        ); 
    }

    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // Immediately switch to the higher-priority task after the ISR
    }
}
