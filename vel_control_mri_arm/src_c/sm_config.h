#include <stdio.h>

#ifndef __SM_CONFIG_H
#define __SM_CONFIG_H

// Set ModeMachine behavior mode: 0 for IDLE, 1 for HOME, 2 for RUN
typedef enum {
    SET_IDLE = 0,
    SET_RUN = 2,
    SET_ERROR = 3
  } MODE_ENUM;

#endif /* __SM_CONFIG_H */


