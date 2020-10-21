/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit GarageDoor Project
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_event.h>
#include <esp_log.h>
#include "driver/gpio.h"

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>
#include <homekit_states.h>

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "GARDOOR";

#define FAN_TASK_PRIORITY  1
#define FAN_TASK_STACKSIZE 4 * 1024
#define FAN_TASK_NAME      "hap_garage"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<CONFIG_GPIO_OUTPUT_IO_RELAY)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<CONFIG_GPIO_INPUT_IO_OPEN) | (1ULL<<CONFIG_GPIO_INPUT_IO_CLOSE))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
static int default_door_state = CURRENT_STATE_STOPPED;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_changeinput(void* arg)
{
    uint32_t io_num;
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

static void gpio_setup(void)
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

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_GPIO_INPUT_IO_OPEN, gpio_isr_handler, (void*) CONFIG_GPIO_INPUT_IO_OPEN);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_GPIO_INPUT_IO_CLOSE, gpio_isr_handler, (void*) CONFIG_GPIO_INPUT_IO_CLOSE);

    //remove isr handler for gpio number.
    //gpio_isr_handler_remove(GPIO_INPUT_IO_OPEN);
    //hook isr handler for specific gpio pin again
    //gpio_isr_handler_add(GPIO_INPUT_IO_OPEN, gpio_isr_handler, (void*) GPIO_INPUT_IO_OPEN);
}

static void start_gpio(void)
{
    xTaskCreate(gpio_task_changeinput, "gpio_task_changeinput", 2048, NULL, 10, NULL);
}

static void kickrelay(void)
{
    // Check that the GPIO pin is a digital and not ADC!
    gpio_set_level(CONFIG_GPIO_OUTPUT_IO_RELAY, 1);
    vTaskDelay(400 / portTICK_RATE_MS);
    gpio_set_level(CONFIG_GPIO_OUTPUT_IO_RELAY, 0);
}

static uint8_t get_door_current_state(void)
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

static void set_door_target_state(uint8_t target_state)
{
    // If we went to open the door, make sure we are closed first.
    // If we want to close the door, make sure we are open first.
    // We do not want kick the relay otherwise
    uint8_t current_state = get_current_state();
    if ((current_state == CURRENT_STATE_OPEN) && (target_state == TARGET_STATE_CLOSED))
    {
        kickrelay();
    } else if ((current_state == CURRENT_STATE_CLOSED) && (target_state == TARGET_STATE_OPEN))
    {
        kickrelay();
    }
    else
    {
        ESP_LOGW(TAG, "Refusing to kick relay for current state: %d", current_state);
    }
}

/*
 * Special function to close the door if it's open. Used for a special switch to close the door
 * only if it's open. Homekit sometimes loses it's state.
 */
static void close_if_open()
{
    uint8_t current_state = get_current_state();
    if (current_state == CURRENT_STATE_OPEN)
    {
        kickrelay();
    }
}

/*
 * If the door is open or closed, it's not in motion. Otherwise, it is (or stuck).
 */
static bool get_motion_detected(void)
{
    bool inmotion = true;
    int current_state = get_door_current_state();
    if ((current_state == CURRENT_STATE_OPEN) || (current_state == CURRENT_STATUS_CLOSE))
    {
        inmotion = false;
    }
}

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int garage_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void garage_hap_event_handler(void* arg, esp_event_base_t event_base, int event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* 
 * In an actual accessory, this should read from hardware.
 * Read routines are generally not required as the value is available with th HAP core
 * when it is updated from write routines. For external triggers (like fan switched on/off
 * using physical button), accessories should explicitly call hap_char_update_val()
 * instead of waiting for a read request.
 */
static int garage_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
       /* Read the current value, toggle it and set the new value.
        * A separate variable should be used for the new value, as the hap_char_get_val()
        * API returns a const pointer
        */
        const hap_val_t *cur_val = hap_char_get_val(hc);

        hap_val_t new_val;
        if (cur_val->i == 1) {
            new_val.i = 0;
        } else {
            new_val.i = 1;
        }
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE)) 
    {
        cur_val->i = 
        if ()
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
    }

    return HAP_SUCCESS;
}

/* A dummy callback for handling a write on the "On" characteristic of Fan.
 * In an actual accessory, this should control the hardware
 */
static int garage_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "Received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "GarageDoor Write called with %d chars", count);
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
            ESP_LOGI(TAG, "Received Write TargetDoorState: %d", write->val.b);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

/*The main thread for handling the GarageDoor Accessory */
static void garage_thread_entry(void *p)
{
    hap_acc_t *garageaccessory(NULL);
    hap_serv_t *garagedooorservice(NULL), *opencontactsensorservice(NULL)

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Esp-GarageDoor",
        .manufacturer = "Espressif",
        .model = "EspGarageDoor01",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = garage_identify,
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
    };
    /* Create accessory object */
    garageaccessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(garageaccessory, product_data, sizeof(product_data));

    uint8_t currentdoorstate = get_door_current_state();
    /* Create the GarageDoor Service. Include the "name" since this is a user visible service  */
    garagedooorservice = hap_serv_garage_door_opener_create(currentdoorstate, CURRENT_STATE_CLOSED, false);
    hap_serv_add_char(garagedooorservice, hap_char_name_create("ESP Garage Door"));
    hap_serv_add_char(garagedooorservice, hap_char_target_door_state_create(0));
    hap_serv_add_char(garagedooorservice, hap_char_current_door_state_create(0));
    /* Set the write callback for the service */
    hap_serv_set_write_cb(garagedooorservice, garage_write);
    /* Set the read callback for the service (optional) */
    hap_serv_set_read_cb(garagedooorservice, garage_read);
    /* Add the Garage Service to the Accessory Object */
    hap_acc_add_serv(garageaccessory, garagedooorservice);

    hap_serv_contact_sensor_create
    //hap_serv_switch_create

    /* Create the Garage Door Contact Sensors */
    //hap_serv_add_char(service, hap_char_contact_sensor_state_create(0));
    //hap_serv_add_char(service, hap_char_contact_sensor_state_create(1));
    //hap_serv_add_char(service, hap_char_motion_detected_create(false));


    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    //hap_fw_upgrade_config_t ota_config = {
    //    .server_cert_pem = server_cert,
    //};
    //service = hap_serv_fw_upgrade_create(&ota_config);
    /* Add the service to the Accessory Object */
    //hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    //hap_add_accessory(accessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* Register an event handler for HomeKit specific events */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &garage_hap_event_handler, NULL);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

    /* Setup the gpio pins */

    gpio_setup();

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_HOMEKIT_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_HOMEKIT_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_HOMEKIT_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_HOMEKIT_SETUP_CODE, CONFIG_HOMEKIT_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_HOMEKIT_SETUP_CODE, CONFIG_HOMEKIT_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* After all the initializations are done, start the HAP core */
    hap_start();

    start_gpio();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    
    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    xTaskCreate(garage_thread_entry, FAN_TASK_NAME, FAN_TASK_STACKSIZE, NULL, FAN_TASK_PRIORITY, NULL);
}
