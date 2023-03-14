#ifndef FEATURE_FIFO_H
#define FEATURE_FIFO_H

#include <stdint.h>

#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* FIFO Buffer Size (Model input * 2) */
#define FIFO_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

void feature_fifo_push(float* feature);
void feature_fifo_get(float* buffer, uint16_t length);
uint16_t feature_fifo_get_count();

#endif  // FEATURE_FIFO_H
