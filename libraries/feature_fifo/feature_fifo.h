#ifndef FEATURE_FIFO_H
#define FEATURE_FIFO_H

#include <stdint.h>

void feature_fifo_push(float* feature);
void feature_fifo_get(float* buffer, uint16_t length);
uint16_t feature_fifo_get_count();

#endif  // FEATURE_FIFO_H
