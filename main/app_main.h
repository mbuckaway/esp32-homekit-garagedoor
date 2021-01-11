#pragma once

#include <stdlib.h>

void motion_sensor_update(bool state);
void close_sensor_update(uint8_t state);
void open_sensor_update(uint8_t state);
void door_switches_update(bool state);
void door_status_update(int state);
void reset_to_factory_handler(void);
