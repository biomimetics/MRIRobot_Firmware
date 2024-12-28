#ifndef QDEC_H
#define QDEC_H

#include <zephyr.h>
#include <math.h>

int read_deg3(void);
int read_deg5(void);
void reset_deg();
int check_count(uint32_t curr, uint32_t last);

#endif