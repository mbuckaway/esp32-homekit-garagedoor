#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "homekit_states.h"

static char *garagedoor_current_states[] = {
    "open",
    "closed",
    "opening",
    "closing",
    "stopped"
};

static char *garagedoor_target_states[] = {
    "open",
    "close"
};

static char *contact_state[] =
{
    "contact detected",
    "contact not detected"
};

/**
 * @brief Change the Garage Door current status to a string
 *
 * @param(status) - current status
 * 
 * @return string to the name of the status
 */

char *garagedoor_current_state_string(uint8_t state)
{
    if (state>CURRENT_STATE_STOPPED)
    {
        state = CURRENT_STATE_OPEN;
    }
    return (garagedoor_current_states[state]);
}

/**
 * @brief Change the Garage Door target status to a string
 *
 * @param(status) - target status
 * 
 * @return string to the name of the status
 */

char *garagedoor_target_state_string(uint8_t state)
{
    if (state>TARGET_STATE_CLOSED)
    {
        state = TARGET_STATE_CLOSED;
    }
    return (garagedoor_target_states[state]);
}

/**
 * @brief Change the contact sensor status to a string
 *
 * @param(status) - contact status
 * 
 * @return string to the name of the status
 */

char *contact_state_string(uint8_t state)
{
    if (state>CONTACT_NOT_DETECTED)
    {
        state = CONTACT_NOT_DETECTED;
    }
    return (contact_state[state]);
}