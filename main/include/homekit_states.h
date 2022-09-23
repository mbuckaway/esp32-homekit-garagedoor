#pragma once

#include <stdlib.h>

/**
 * TargetDoorState enum - matches homekit status for same
    Characteristic.TargetDoorState.OPEN = 0;
    Characteristic.TargetDoorState.CLOSED = 1;
 */
enum TargetDoorState {
    TARGET_STATE_OPEN = 0,
    TARGET_STATE_CLOSED = 1
};

/**
 * CurrentDoorState enum - matches homekit status for same
    Characteristic.CurrentDoorState.OPEN = 0;
    Characteristic.CurrentDoorState.CLOSED = 1;
    Characteristic.CurrentDoorState.OPENING = 2;
    Characteristic.CurrentDoorState.CLOSING = 3;
    Characteristic.CurrentDoorState.STOPPED = 4;
 */
enum CurrentDoorState {
    CURRENT_STATE_OPEN = 0,
    CURRENT_STATE_CLOSED = 1,
    CURRENT_STATE_OPENING = 2,
    CURRENT_STATE_CLOSING = 3,
    CURRENT_STATE_STOPPED = 4
};

/**
 * ContactState enum - matches the homekit status for same
 * Characteristic.ContactSensorState.CONTACT_DETECTED = 0;
 * Characteristic.ContactSensorState.CONTACT_NOT_DETECTED = 1;
 */
enum ContactState {
    CONTACT_DETECTED = 0,
    CONTACT_NOT_DETECTED = 1
};

char *garagedoor_current_state_string(uint8_t state);
char *garagedoor_target_state_string(uint8_t state);
char *contact_state_string(uint8_t state);