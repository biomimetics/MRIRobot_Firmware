#include <stdio.h>

#ifndef __SM_CONFIG_H
#define __SM_CONFIG_H

// Set ModeMachine behavior mode: 0 for IDLE, 1 for HOME, 2 for RUN
typedef enum {
    SET_IDLE = 0,
    SET_HOMING = 1,
    SET_RUN_POS = 2,
    SET_RUN_VEL = 3
  } MODE_ENUM;

#endif /* __SM_CONFIG_H */


#ifndef __MOTOR_MODE_ENUM
#define __MOTOR_MODE_ENUM

  // Set Motor Mode -> velocity or position control
  typedef enum {
      MODE_POS = 0U,
      MODE_VEL = 1U,
    } MOTOR_MODE;
  

  // Set Target Mode -> Internal or external control
  typedef enum {
      MODE_INT = 0U,
      MODE_EXT = 1U,
    } TARGET_SRC;

#endif // __MOTOR_MODE_ENUM