#ifndef LSM6DS3_SIMPLE_H
#define LSM6DS3_SIMPLE_H

#include <stdint.h>

int lsm6ds3_simple_init(void);
int lsm6ds3_simple_read_accel(int16_t *x, int16_t *y, int16_t *z);
int lsm6ds3_simple_read_temp(int16_t *temp);

#endif
