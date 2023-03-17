#include "feature_fifo.h"

static float feature_buffer[FIFO_BUFFER_SIZE];
static uint16_t fifo_end_index = 0;

void feature_fifo_push(float* feature) {
  memcpy(&(feature_buffer[fifo_end_index]), feature,
         EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME * sizeof(float));
  fifo_end_index += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
  if (fifo_end_index == FIFO_BUFFER_SIZE) {
    memcpy(feature_buffer,
           &feature_buffer[fifo_end_index - EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE],
           EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));
    fifo_end_index = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  }
}

void feature_fifo_get(float* buffer, uint16_t length) {
  uint16_t start_index = fifo_end_index - length;
  memcpy(buffer, &(feature_buffer[start_index]), length * sizeof(float));
}

uint16_t feature_fifo_get_count() {
  return fifo_end_index <= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
             ? fifo_end_index
             : FIFO_BUFFER_SIZE;
}
