 /* interface for quadrature decoder in STM32F4 timer peripheral*/
#include "encoder.h"
#include <sys/printk.h>
#include <soc.h>


// Variables to keep track of rotations
uint32_t last_cnt;
int rotations;

int check_count(uint32_t curr, uint32_t last){
	if (last > 60000 && curr <= 3000) {
		return 1;
	} else if (curr > 60000 && last <= 3000) {
		return -1;
	}
	return 0;
}

int read_deg3()
{
	int32_t qdec_val = read_qdec3();
	
	rotations += check_count(qdec_val, last_cnt);
	last_cnt = qdec_val;

	// return (((double)rotations * 360 * 65535) / 23000) + ((((double)qdec_val) * 360) / 23000);
	return rotations * 65535 + qdec_val;
	// if (reading > 32766) {
	// 	return -1 * (((65532 - read_qdec()) * 360) / 23000);
	// }
	// else {
	// 	return ((read_qdec()) * 360) / 23000;
	// }
}

int read_deg5()
{
	int32_t qdec_val = read_qdec5();

	return qdec_val;
}

void reset_deg() {
	reset_qdec3();
	reset_qdec5();
	rotations = 0;
}

