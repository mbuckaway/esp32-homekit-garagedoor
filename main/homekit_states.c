#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "homekit_states.h"

static char *garagedoor_states[] = {
    "open",
    "closed",
    "opening",
    "closing",
    "stopped"
};

/**
 * @brief Change the Garage Door open status to a string
 *
 * @param(status) - current status
 * 
 * @return string to the name of the status
 */

char *garagedoor_status_string(uint8_t status)
{
    if (status>CURRENT_STATE_STOPPED)
    {
        status = CURRENT_STATE_OPEN;
    }
    return (garagedoor_states[status]);
}