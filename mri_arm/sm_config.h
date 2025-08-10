#include <stdio.h>

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

#ifndef __SM_CONFIG_H
#define __SM_CONFIG_H

// Set ModeMachine behavior mode: 0 for IDLE, 1 for HOME, 2 for RUN
typedef enum {
    SET_IDLE = 0,
    SET_HOMING = 1,
    SET_RUN_POS = 2,
    SET_RUN_VEL = 3
  } MODE_ENUM;

typedef struct {
    MODE_ENUM current_mode_index;
    bool is_homing_finished;
    bool enable_homing_command;
    bool enable_motor_command;
    bool force_disable_motor_pins;
    TARGET_SRC motor_controller_internal_external_target_mode;
    MOTOR_MODE motor_controller_target_sel[7];
  } mode_status;
#endif /* __SM_CONFIG_H */


