#include "limitSwitch.h"

#include <stddef.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#define LINK1_LEFT_LIMIT_GPIO GPIO_NUM_4
#define LINK1_RIGHT_LIMIT_GPIO GPIO_NUM_6
#define LINK2_LEFT_LIMIT_GPIO GPIO_NUM_15
#define LINK2_RIGHT_LIMIT_GPIO GPIO_NUM_17
#define LIMIT_SWITCH_DEBOUNCE_MS 25

static void limitSwitchTask(void *arg);
static void limitSwitchISR(void *arg);

static volatile bool link1LeftSwitchPressed = false;
static volatile bool link1RightSwitchPressed = false;
static volatile bool link2LeftSwitchPressed = false;
static volatile bool link2RightSwitchPressed = false;
static volatile bool anySwitchPressed = false;
static int previousLink1LeftState = 1;
static int previousLink1RightState = 1;
static int previousLink2LeftState = 1;
static int previousLink2RightState = 1;
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
            (1ULL << LINK1_RIGHT_LIMIT_GPIO) |
            (1ULL << LINK2_LEFT_LIMIT_GPIO) |
            (1ULL << LINK2_RIGHT_LIMIT_GPIO),
            // << means, moving that 1(ON) sign to the left multiple times (# of gpio number)
            // bit mask is used because same config can also be applied to multiple gpio if wanted
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&switchConfig); // applying the above gpio configuration

    // ------------------------- Initial State Check ----------------------------
        
    previousLink1LeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO); //get_level to get the value
    previousLink1RightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);
    previousLink2LeftState = gpio_get_level(LINK2_LEFT_LIMIT_GPIO);
    previousLink2RightState = gpio_get_level(LINK2_RIGHT_LIMIT_GPIO);

    // (1 = Unpressed as default state)
    // (0 = Pressed, because the limit switch is connected to GND when pressed)
    link1LeftSwitchPressed = previousLink1LeftState == 0;
    link1RightSwitchPressed = previousLink1RightState == 0;
    link2LeftSwitchPressed = previousLink2LeftState == 0;
    link2RightSwitchPressed = previousLink2RightState == 0;
    anySwitchPressed = link1LeftSwitchPressed || link1RightSwitchPressed ||
                       link2LeftSwitchPressed || link2RightSwitchPressed;
        // if either previous state is 0, then it means the limit switch is newly pressed

    xTaskCreate(limitSwitchTask, "limitSwitchTask", 4096, NULL, 10, &limitSafetyTaskHandle); 
        // Priority 10 (Higher than others)

    // ------------------------- Interrupt Configuration --------------------------
    gpio_isr_handler_add(LINK1_LEFT_LIMIT_GPIO, limitSwitchISR, NULL);
    gpio_isr_handler_add(LINK1_RIGHT_LIMIT_GPIO, limitSwitchISR, NULL);
    gpio_isr_handler_add(LINK2_LEFT_LIMIT_GPIO, limitSwitchISR, NULL);
    gpio_isr_handler_add(LINK2_RIGHT_LIMIT_GPIO, limitSwitchISR, NULL);

    gpio_set_intr_type(LINK1_LEFT_LIMIT_GPIO, GPIO_INTR_ANYEDGE); // Any change will trigger the interrupt  
    gpio_set_intr_type(LINK1_RIGHT_LIMIT_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(LINK2_LEFT_LIMIT_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(LINK2_RIGHT_LIMIT_GPIO, GPIO_INTR_ANYEDGE);

    link1LeftSwitchPressed = gpio_get_level(LINK1_LEFT_LIMIT_GPIO) == 0;
    link1RightSwitchPressed = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO) == 0;
    link2LeftSwitchPressed = gpio_get_level(LINK2_LEFT_LIMIT_GPIO) == 0;
    link2RightSwitchPressed = gpio_get_level(LINK2_RIGHT_LIMIT_GPIO) == 0;
    anySwitchPressed = link1LeftSwitchPressed || link1RightSwitchPressed ||
                       link2LeftSwitchPressed || link2RightSwitchPressed;
 
    // ------------------------- Initial Notification --------------------------
    if (anySwitchPressed) {
        xTaskNotifyGive(limitSafetyTaskHandle);
        /*“If a limit switch is already pressed by the time it initiates, give the task 
        one notification so the first loop iteration checks and stops the motors.”*/
    } 
}

bool limitSwitchAnyIsPressed(void)
{
    return anySwitchPressed;
}
bool limitSwitchLink1LeftIsPressed(void)
{
    return link1LeftSwitchPressed;
}
bool limitSwitchLink1RightIsPressed(void)
{
    return link1RightSwitchPressed;
}
bool limitSwitchLink2LeftIsPressed(void)
{
    return link2LeftSwitchPressed;
}
bool limitSwitchLink2RightIsPressed(void)
{
    return link2RightSwitchPressed;
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
            int currentLink1LeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            int currentLink1RightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);
            int currentLink2LeftState = gpio_get_level(LINK2_LEFT_LIMIT_GPIO);
            int currentLink2RightState = gpio_get_level(LINK2_RIGHT_LIMIT_GPIO);

            //------------------------- Emergency Stop --------------------------------
            // 0 is pressed, 1 is unpressed for left and right limit switches (because they are connected to GND when pressed)
            // 0 is unpressed, 1 is pressed for anySwitchPressed (because it is a logical)
            if ((currentLink1LeftState == 0) || (currentLink1RightState == 0) ||
                (currentLink2LeftState == 0) || (currentLink2RightState == 0) || anySwitchPressed) {
                anySwitchPressed = true;

                if (onLimitSwitchPressed != NULL) {
                    // <<<<<<<<<<<<<<<<<<EMERGENCY STOP FUNCTION POINTER (motorEmergencyStop) IS CALLED HERE>>>>>>>>>>>>>>>>
                    onLimitSwitchPressed(); // == motor.motorEmergencyStop();  
                }
            }

            //------------------------ Print (PRESSED) ---------------------------
            if ((currentLink1LeftState == 0) && (previousLink1LeftState != 0)) {
                link1LeftSwitchPressed = true;
                printf("Link 1 Left limit switch PRESSED\n");
                previousLink1LeftState = 0;
            }

            if ((currentLink1RightState == 0) && (previousLink1RightState != 0)) {
                link1RightSwitchPressed = true;
                printf("Link 1 Right limit switch PRESSED\n");
                previousLink1RightState = 0;
            }

            if ((currentLink2LeftState == 0) && (previousLink2LeftState != 0)) {
                link2LeftSwitchPressed = true;
                printf("Link 2 Left limit switch PRESSED\n");
                previousLink2LeftState = 0;
            }

            if ((currentLink2RightState == 0) && (previousLink2RightState != 0)) {
                link2RightSwitchPressed = true;
                printf("Link 2 Right limit switch PRESSED\n");
                previousLink2RightState = 0;
            }

            //------------------------- Debouncing ----------------------------
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LIMIT_SWITCH_DEBOUNCE_MS)) != 0) {
                continue;
            }

            currentLink1LeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            currentLink1RightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);
            currentLink2LeftState = gpio_get_level(LINK2_LEFT_LIMIT_GPIO);
            currentLink2RightState = gpio_get_level(LINK2_RIGHT_LIMIT_GPIO);

            //------------------------ Print (RELEASED) ---------------------------
            if ((currentLink1LeftState != previousLink1LeftState) && (currentLink1LeftState != 0)) {
                printf("Link 1 Left limit switch RELEASED\n");
                previousLink1LeftState = currentLink1LeftState;
            }

            if ((currentLink1RightState != previousLink1RightState) && (currentLink1RightState != 0)) {
                printf("Link 1 Right limit switch RELEASED\n");
                previousLink1RightState = currentLink1RightState;
            }

            if ((currentLink2LeftState != previousLink2LeftState) && (currentLink2LeftState != 0)) {
                printf("Link 2 Left limit switch RELEASED\n");
                previousLink2LeftState = currentLink2LeftState;
            }

            if ((currentLink2RightState != previousLink2RightState) && (currentLink2RightState != 0)) {
                printf("Link 2 Right limit switch RELEASED\n");
                previousLink2RightState = currentLink2RightState;
            }

            link1LeftSwitchPressed = currentLink1LeftState == 0;
            link1RightSwitchPressed = currentLink1RightState == 0;
            link2LeftSwitchPressed = currentLink2LeftState == 0;
            link2RightSwitchPressed = currentLink2RightState == 0;
            anySwitchPressed = link1LeftSwitchPressed || link1RightSwitchPressed ||
                               link2LeftSwitchPressed || link2RightSwitchPressed;
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
        (gpio_get_level(LINK1_RIGHT_LIMIT_GPIO) == 0) ||
        (gpio_get_level(LINK2_LEFT_LIMIT_GPIO) == 0) ||
        (gpio_get_level(LINK2_RIGHT_LIMIT_GPIO) == 0);

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
