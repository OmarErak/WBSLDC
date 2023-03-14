/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _EI_CLASSIFIER_PORTING_H_
#define _EI_CLASSIFIER_PORTING_H_

#include <stdint.h>
#include <stdlib.h>

#include "edge-impulse-sdk/tensorflow/lite/micro/debug_log.h"

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
#endif  // defined(__cplusplus)

typedef enum {
  EI_IMPULSE_OK = 0,
  EI_IMPULSE_ERROR_SHAPES_DONT_MATCH = -1,
  EI_IMPULSE_CANCELED = -2,
  EI_IMPULSE_TFLITE_ERROR = -3,
  EI_IMPULSE_DSP_ERROR = -5,
  EI_IMPULSE_TFLITE_ARENA_ALLOC_FAILED = -6,
  EI_IMPULSE_CUBEAI_ERROR = -7,
  EI_IMPULSE_ALLOC_FAILED = -8,
  EI_IMPULSE_ONLY_SUPPORTED_FOR_IMAGES = -9,
  EI_IMPULSE_UNSUPPORTED_INFERENCING_ENGINE = -10,
  EI_IMPULSE_OUT_OF_MEMORY = -11,
  EI_IMPULSE_INPUT_TENSOR_WAS_NULL = -13,
  EI_IMPULSE_OUTPUT_TENSOR_WAS_NULL = -14,
  EI_IMPULSE_SCORE_TENSOR_WAS_NULL = -15,
  EI_IMPULSE_LABEL_TENSOR_WAS_NULL = -16,
  EI_IMPULSE_TENSORRT_INIT_FAILED = -17,
  EI_IMPULSE_DRPAI_INIT_FAILED = -18,
  EI_IMPULSE_DRPAI_RUNTIME_FAILED = -19,
  EI_IMPULSE_DEPRECATED_MODEL = -20,
  EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE = -21,
  EI_IMPULSE_INFERENCE_ERROR = -22,
  EI_IMPULSE_AKIDA_ERROR = -23,
  EI_IMPULSE_INVALID_SIZE = -24,
  EI_IMPULSE_ONNX_ERROR = -25,
} EI_IMPULSE_ERROR;

/**
 * Cancelable sleep, can be triggered with signal from other thread
 */
EI_IMPULSE_ERROR ei_sleep(int32_t time_ms);

/**
 * Check if the sampler thread was canceled, use this in conjunction with
 * the same signaling mechanism as ei_sleep
 */
EI_IMPULSE_ERROR ei_run_impulse_check_canceled();

/**
 * Read the millisecond timer
 */
uint64_t ei_read_timer_ms();

/**
 * Read the microsecond timer
 */
uint64_t ei_read_timer_us();

/**
 * Set Serial baudrate
 */
void ei_serial_set_baudrate(int baudrate);

/**
 * @brief      Connect to putchar of target
 *
 * @param[in]  c The chararater
 */
void ei_putchar(char c);

/**
 * Print wrapper around printf()
 * This is used internally to print debug information.
 */
__attribute__((format(printf, 1, 2))) void ei_printf(const char *format, ...);

/**
 * Override this function if your target cannot properly print floating points
 * If not overriden, this will be sent through `ei_printf()`.
 */
void ei_printf_float(float f);

/**
 * Wrapper around malloc
 */
void *ei_malloc(size_t size);

/**
 * Wrapper around calloc
 */
void *ei_calloc(size_t nitems, size_t size);

/**
 * Wrapper around free
 */
void ei_free(void *ptr);

#if defined(__cplusplus) && EI_C_LINKAGE == 1
}
#endif  // defined(__cplusplus) && EI_C_LINKAGE == 1

#endif  // _EI_CLASSIFIER_PORTING_H_
