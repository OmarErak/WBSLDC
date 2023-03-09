#include "edge-impulse-sdk/dsp/config.hpp"
#if EIDSP_LOAD_CMSIS_DSP_SOURCES

/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_rogerstanimoto_distance.c
 * Description:  Roger Stanimoto distance between two vectors
 *
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2019 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <limits.h>
#include <math.h>

#include "edge-impulse-sdk/CMSIS/DSP/Include/dsp/distance_functions.h"

extern void arm_boolean_distance_TT_FF_TF_FT(const uint32_t *pA,
                                             const uint32_t *pB,
                                             uint32_t numberOfBools,
                                             uint32_t *cTT, uint32_t *cFF,
                                             uint32_t *cTF, uint32_t *cFT);

/**
  @addtogroup BoolDist
  @{
 */

/**
 * @brief        Rogers Tanimoto distance between two vectors
 *
 * @param[in]    pA              First vector of packed booleans
 * @param[in]    pB              Second vector of packed booleans
 * @param[in]    numberOfBools   Number of booleans
 * @return distance
 *
 */

float32_t arm_rogerstanimoto_distance(const uint32_t *pA, const uint32_t *pB,
                                      uint32_t numberOfBools) {
  uint32_t ctt = 0, cff = 0, ctf = 0, cft = 0, r;

  arm_boolean_distance_TT_FF_TF_FT(pA, pB, numberOfBools, &ctt, &cff, &ctf,
                                   &cft);

  r = 2 * (ctf + cft);

  return (1.0 * r / (r + ctt + cff));
}

/**
 * @} end of BoolDist group
 */

#endif  // EIDSP_LOAD_CMSIS_DSP_SOURCES
