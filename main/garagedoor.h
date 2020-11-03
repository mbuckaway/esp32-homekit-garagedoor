#pragma once

#include <stdlib.h>
#include <stdio.h>

void garagedoor_setup(void);
void start_garagedoor(void);
void kickrelay(void);
uint8_t get_door_current_state(void);
void set_door_target_state(uint8_t target_state);
void force_door_target_state(uint8_t target_state);
bool get_motion_detected(void);
uint8_t get_open_contact_status(void);
uint8_t get_close_contact_status(void);
