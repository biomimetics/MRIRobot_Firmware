#include <stdio.h>

#ifndef __SM_CONFIG_H
#define __SM_CONFIG_H

// Set ModeMachine behavior mode: 0 for IDLE, 1 for HOME, 2 for RUN
// NOTE: this enum's values don't currently match what State_Machine.lf's
// reaction(command_message) actually switches on (raw 0/1/2/3 there) -- kept
// here for reference only, not included by State_Machine.lf.
typedef enum {
    SET_IDLE = 0,
    SET_RUN = 2,
    SET_ERROR = 3
  } MODE_ENUM;

#endif /* __SM_CONFIG_H */


