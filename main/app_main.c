/*
 * Copyright (c) 2020 <Mark Buckaway> MIT License
 * 
 * HomeKit GarageDoor Project
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_event.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

//#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "garagedoor.h"
#include "homekit_states.h"

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "HAP";

static const uint16_t GARAGEDOOR_TASK_PRIORITY = 1;
static const uint16_t GARAGEDOOR_TASK_STACKSIZE = 4 * 1024;
static const char *GARAGEDOOR_TASK_NAME = "hap_garage";

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
static const uint16_t RESET_NETWORK_BUTTON_TIMEOUT = 3;

/* Reset to factory if button is pressed and held for more than 10 seconds */
 static const uint16_t RESET_TO_FACTORY_BUTTON_TIMEOUT = 10;

/* The button "Boot" will be used as the Reset button for the example */
static const uint16_t RESET_GPIO = GPIO_NUM_0;


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
static int garagedoor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "garagedoor received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE)) 
    {
        hap_val_t new_val;
        new_val.i = get_door_current_state();
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"garagedoor status updated to %s", garagedoor_current_state_string(new_val.i));
    }
    return HAP_SUCCESS;
}

/* A dummy callback for handling a write on the "On" characteristic of Fan.
 * In an actual accessory, this should control the hardware
 */
static int garagedoor_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "garagedoor received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "garagedoor write called with %d chars", count);
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
            ESP_LOGI(TAG, "garagedoor received write TargetDoorState: %s", garagedoor_target_state_string(write->val.b));
            set_door_target_state(write->val.b);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

/**
 * @brief Reads the status of the garage door to select if the switch should be on or off. We set it to ON
 * if the door it open, and off if its anything else. Used for the door_switch_service and closeif_switch_service.
 */
static int door_switch_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "closeif/door_switch received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_ON)) 
    {
        hap_val_t new_val;
        // use open as on
        new_val.i = (bool)get_door_current_state()==CURRENT_STATE_OPEN;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"closeif/door_switch status updated to %s", (new_val.i)?"on(open)":"off(closed)");
    }
    return HAP_SUCCESS;
}

/**
 * @brief Takes a request from homekit and sets the switch status. We use on for open and off for closed
 */
static int door_switch_write(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "door switch received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "door switch write called with %d chars", count);
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
            ESP_LOGI(TAG, "door switch received write On State: %s", write->val.b?"on(open)":"off(closed)");
            set_door_target_state(write->val.b?TARGET_STATE_OPEN:TARGET_STATE_CLOSED);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

/**
 * @brief Takes a request from homekit and sets the switch status. We use on for open and off for closed
 */
static int closeif_switch_write(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "closeif received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "closeif write called with %d chars", count);
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
            ESP_LOGI(TAG, "closeif received write ON State: %s", write->val.b?"on(open)":"off(closed)");
            force_door_target_state(write->val.b?TARGET_STATE_OPEN:TARGET_STATE_CLOSED);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

static int open_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "open sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CONTACT_SENSOR_STATE)) 
    {
        hap_val_t new_val;
        // use open as on
        new_val.i = get_open_contact_status();
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"open sensor status updated to %s", contact_state_string(new_val.i));
    }
    return HAP_SUCCESS;
}

static int close_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "close sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CONTACT_SENSOR_STATE)) 
    {
        hap_val_t new_val;
        // use open as on
        new_val.i = get_close_contact_status();
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"close sensor status updated to %s", contact_state_string(new_val.i));
    }
    return HAP_SUCCESS;
}

/**
 * @brief Main Thread to handle setting up the service and accessories for the GarageDoor
 */
static void garage_thread_entry(void *p)
{
    hap_acc_t *garageaccessory = NULL;
    hap_serv_t *garagedoorservice = NULL;
    hap_serv_t *closeif_switch_service = NULL;
    hap_serv_t *door_switch_service = NULL;
    hap_serv_t *opencontact_sensor_service = NULL;
    hap_serv_t *closecontact_sensor_service = NULL;

    /*
     * Configure the GPIO for the garage door state/relay control
     */
    garagedoor_setup();

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    ESP_LOGI(TAG, "configuring HAP");
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    ESP_LOGI(TAG, "initializing HAP");
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
        .hw_rev =  (char*)esp_get_idf_version(),
        .pv = "1.1.0",
        .identify_routine = garage_identify,
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
    };
    ESP_LOGI(TAG, "Creating garage door accessory...");
    /* Create accessory object */
    garageaccessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(garageaccessory, product_data, sizeof(product_data));

    uint8_t currentdoorstate = get_door_current_state();
    
    ESP_LOGI(TAG, "Creating garage door service (current state: %s)", garagedoor_current_state_string(currentdoorstate));

    /* Create the GarageDoor Service. Include the "name" since this is a user visible service  */
    garagedoorservice = hap_serv_garage_door_opener_create(currentdoorstate, TARGET_STATE_CLOSED, false);
    hap_serv_add_char(garagedoorservice, hap_char_name_create("ESP Garage Door"));
    //hap_serv_add_char(garagedoorservice, hap_char_target_door_state_create(currentdoorstate));
    //hap_serv_add_char(garagedoorservice, hap_char_current_door_state_create(currentdoorstate));
    /* Set the write callback for the service */
    hap_serv_set_write_cb(garagedoorservice, garagedoor_write);
    /* Set the read callback for the service (optional) */
    hap_serv_set_read_cb(garagedoorservice, garagedoor_read);
    /* Add the Garage Service to the Accessory Object */
    hap_acc_add_serv(garageaccessory, garagedoorservice);

    ESP_LOGI(TAG, "Creating closeif service (current state: %s)", (currentdoorstate==CURRENT_STATE_OPEN)?"on(open)":"off(closed)");
    // Create the CloseIf switch
    closeif_switch_service = hap_serv_switch_create((bool)(currentdoorstate==CURRENT_STATE_OPEN));
    hap_serv_add_char(closeif_switch_service, hap_char_name_create("ESP CloseIf Door"));
    //hap_serv_add_char(closeif_switch_service, hap_char_on_create((bool)(currentdoorstate==CURRENT_STATE_OPEN)));
    hap_serv_set_read_cb(closeif_switch_service, door_switch_read);
    hap_serv_set_write_cb(closeif_switch_service, closeif_switch_write);
    hap_acc_add_serv(garageaccessory, closeif_switch_service);

    ESP_LOGI(TAG, "Creating door switch service (current state: %s)", (currentdoorstate==CURRENT_STATE_OPEN)?"on(open)":"off(closed)");
    // Create the CloseIf switch
    door_switch_service = hap_serv_switch_create((bool)(currentdoorstate==CURRENT_STATE_OPEN));
    hap_serv_add_char(door_switch_service, hap_char_name_create("ESP Door Switch"));
    //hap_serv_add_char(door_switch_service, hap_char_on_create((bool)(currentdoorstate==CURRENT_STATE_OPEN)));
    hap_serv_set_read_cb(door_switch_service, door_switch_read);
    hap_serv_set_write_cb(door_switch_service, door_switch_write);
    hap_acc_add_serv(garageaccessory, door_switch_service);

    // Create the open contact sensor
    uint8_t openstate = get_open_contact_status();
    ESP_LOGI(TAG, "Creating open contact service (current state: %s)", contact_state_string(openstate));
    opencontact_sensor_service = hap_serv_contact_sensor_create(openstate);
    hap_serv_add_char(opencontact_sensor_service, hap_char_name_create("ESP Open Contact Sensor"));
    hap_serv_set_read_cb(opencontact_sensor_service, open_sensor_read);
    hap_acc_add_serv(garageaccessory, opencontact_sensor_service);

    // Create the open contact sensor
    uint8_t closestate = get_open_contact_status();
    ESP_LOGI(TAG, "Creating close contact service (current state: %s)", contact_state_string(closestate));
    closecontact_sensor_service = hap_serv_contact_sensor_create(get_close_contact_status());
    hap_serv_add_char(closecontact_sensor_service, hap_char_name_create("ESP Close Contact Sensor"));
    hap_serv_set_read_cb(closecontact_sensor_service, close_sensor_read);
    hap_acc_add_serv(garageaccessory, closecontact_sensor_service);

#if 0
    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    service = hap_serv_fw_upgrade_create(&ota_config);
    /* Add the service to the Accessory Object */
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    ESP_LOGI(TAG, "Adding Garage Door Accessory...");
    hap_add_accessory(garageaccessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    ESP_LOGI(TAG, "Register reset GPIO (reset button) on pin %d", RESET_GPIO);
    reset_key_init(RESET_GPIO);

    /* Register an event handler for HomeKit specific events */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &garage_hap_event_handler, NULL);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers", hap_get_paired_controller_count());


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

    /* mfi is not supported */
    hap_enable_mfi_auth(HAP_MFI_AUTH_NONE);

    ESP_LOGI(TAG, "Starting WIFI...");
    /* Initialize Wi-Fi */
    app_wifi_init();

    start_garagedoor();

    /* After all the initializations are done, start the HAP core */
    ESP_LOGI(TAG, "Starting HAP...");
    hap_start();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    
    ESP_LOGI(TAG, "HAP initialization complete.");

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup...");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "[APP] Creating main thread...");

    xTaskCreate(garage_thread_entry, GARAGEDOOR_TASK_NAME, GARAGEDOOR_TASK_STACKSIZE, NULL, GARAGEDOOR_TASK_PRIORITY, NULL);
}
