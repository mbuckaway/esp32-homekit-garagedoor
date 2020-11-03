#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_event.h>
#include <esp_log.h>
#include "driver/gpio.h"

#include "homekit_states.h"


static const char *TAG = "GDGPIO";

static xQueueHandle gpio_evt_queue = NULL;
static uint8_t default_door_state = CURRENT_STATE_STOPPED;

static const long long unsigned int GPIO_OUTPUT_PIN_SEL = (1ULL<<CONFIG_GPIO_OUTPUT_IO_RELAY);
static const long long unsigned int GPIO_INPUT_PIN_SEL = ((1ULL<<CONFIG_GPIO_INPUT_IO_OPEN) | (1ULL<<CONFIG_GPIO_INPUT_IO_CLOSE));
static const uint16_t ESP_INTR_FLAG_DEFAULT = 0;

// Handles the interrupts from the door switches and sends a Queue update along
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Runs a separate thread to process door switch updates. All this does is change
// the default door state. We run when the switch opens, so we indicate when the door is moving
// The status checks check the door status for open or close and not here
static void gpio_task_changeinput(void* arg)
{
    uint32_t io_num;
    ESP_LOGI(TAG, "Garage door task running");
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // If close switch changed, we are opening
            if (io_num == CONFIG_GPIO_INPUT_IO_CLOSE) {
                default_door_state = CURRENT_STATE_OPENING;
            } else if (io_num == CONFIG_GPIO_INPUT_IO_OPEN)
            {
                default_door_state = CURRENT_STATE_CLOSING;
            }
            // It is not possible to have both switches active at once
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

/**
 * @brief Setup the GPIO, ISR, and event queue
 */

void garagedoor_setup(void)
{
    gpio_config_t io_out_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0
    };

    gpio_config(&io_out_conf);

    // Setup an interrupt when the open/close sensors register an open switch state
    // which means the door is opening or closeing
    gpio_config_t io_in_conf = {
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 1
    };

    gpio_config(&io_in_conf);

    // Allow ISR per pin to operate
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_GPIO_INPUT_IO_OPEN, gpio_isr_handler, (void*) CONFIG_GPIO_INPUT_IO_OPEN);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_GPIO_INPUT_IO_CLOSE, gpio_isr_handler, (void*) CONFIG_GPIO_INPUT_IO_CLOSE);

    ESP_LOGI(TAG, "Garage door GPIO configured");
}

/**
 * @brief Starts the garage door thread to watch the event queue
 */
void start_garagedoor(void)
{
    ESP_LOGI(TAG, "Garage door task started");
    xTaskCreate(gpio_task_changeinput, "gpio_task_changeinput", 2048, NULL, 10, NULL);
}

void kickrelay(void)
{
    ESP_LOGI(TAG, "Kicking the relay...");
    // Check that the GPIO pin is a digital and not ADC!
    gpio_set_level(CONFIG_GPIO_OUTPUT_IO_RELAY, 1);
    vTaskDelay(400 / portTICK_RATE_MS);
    gpio_set_level(CONFIG_GPIO_OUTPUT_IO_RELAY, 0);
}

/**
 * @brief Gets the current door state
 *
 * @return u_int8_t representing the door state (open, closed, opening, closing, etc.)
 */
uint8_t get_door_current_state(void)
{
    uint8_t current_state = default_door_state;
    uint8_t open = gpio_get_level(CONFIG_GPIO_INPUT_IO_OPEN);
    uint8_t close = gpio_get_level(CONFIG_GPIO_INPUT_IO_CLOSE);
    if (open)
    {
        current_state = CURRENT_STATE_OPEN;
    } else if (close) {
        current_state = CURRENT_STATE_CLOSED;
    }
    return current_state;

}

/**
 * @brief Get the contact state for the open sensor
 * 
 * @return CONTACT_DETECTED if closed or CONTACT_NOT_DETECTED if open
 */
 uint8_t get_open_contact_status(void)
 {
    return gpio_get_level(CONFIG_GPIO_INPUT_IO_OPEN)?CONTACT_DETECTED:CONTACT_NOT_DETECTED;
 } 
  
/**
 * @brief Get the contact state for the close sensor
 * 
 * @return CONTACT_DETECTED if closed or CONTACT_NOT_DETECTED if open
 */
 uint8_t get_close_contact_status(void)
 {
    return gpio_get_level(CONFIG_GPIO_INPUT_IO_CLOSE)?CONTACT_DETECTED:CONTACT_NOT_DETECTED;
 } 

/**
 * @brief Sets the door state (open or closed)
 *
 * @param[target_state] Target door state
 */
void set_door_target_state(uint8_t target_state)
{
    // If we went to open the door, make sure we are closed first.
    // If we want to close the door, make sure we are open first.
    // We do not want kick the relay otherwise
    uint8_t current_state = get_door_current_state();
    if ((current_state == CURRENT_STATE_OPEN) && (target_state == TARGET_STATE_CLOSED))
    {
        kickrelay();
    } else if ((current_state == CURRENT_STATE_CLOSED) && (target_state == TARGET_STATE_OPEN))
    {
        kickrelay();
    }
    else
    {
        ESP_LOGW(TAG, "Garage door state is %s and requesting %s: Ignored.", garagedoor_current_state_string(current_state), garagedoor_target_state_string(target_state));
    }
}

/**
 * @brief Special function to close the door if it's open and ignore it if closed. Used for a special switch to close the door
 * only if it's open. Homekit sometimes loses it's state.
 */
void force_door_target_state(uint8_t target_state)
{
    // Ignore anything that isn't TARGET_CLOSED
    if (target_state == TARGET_STATE_CLOSED)
    {
        uint8_t current_state = get_door_current_state();
        if (current_state == CURRENT_STATE_OPEN)
        {
            kickrelay();
        }
    }
}

/**
 * @brief If the door is open or closed, it's not in motion. Otherwise, it is (or stuck).
 * 
 * @return bool if the door is considered in motion (not open or closed)
 */
bool get_motion_detected(void)
{
    bool inmotion = true;
    int current_state = get_door_current_state();
    if ((current_state == CURRENT_STATE_OPEN) || (current_state == CURRENT_STATE_CLOSED))
    {
        inmotion = false;
    }
    return inmotion;
}
