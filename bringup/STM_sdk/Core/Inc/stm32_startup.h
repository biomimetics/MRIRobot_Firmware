
#ifndef __STM32_STARTUP
#define __STM32_STARTUP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
int _write(int file, char *ptr, int len);



#ifdef __cplusplus
}
#endif

#endif
