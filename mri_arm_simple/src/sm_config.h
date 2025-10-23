#include <stdio.h>

#ifndef __SM_CONFIG_H
#define __SM_CONFIG_H

// Set ModeMachine behavior mode: 0 for IDLE, 1 for RUN, 2 for ERROR if something goes bad.
typedef enum {
    SET_IDLE = 0,
    SET_RUN = 1, 
    SET_ERROR = 2 
  } MODE_ENUM;

typedef struct {
    MODE_ENUM current_mode_index;
    bool enable_motor_command;
    bool force_disable_motor_pins;
  } mode_status;
#endif /* __SM_CONFIG_H */


