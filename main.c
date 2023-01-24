/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ant_master.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "feature_fifo.h"
#include "lsm9ds1_reg.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* FIFO Buffer Size (Model input * 2) */
#define FIFO_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

/* Private variables ---------------------------------------------------------*/
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float feature[6];

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint8_t register_address = 0x0F;  // Address of the who am i register to be read

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint16_t reg16 = reg;
  err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, (uint8_t *)&reg16, 1, true);
  if (NRF_SUCCESS != err_code) {
    return 0;
  }
  err_code = nrf_drv_twi_rx(&m_twi, *i2c_address, bufp, len);
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len) {
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint8_t buffer[1 + len];
  memcpy(buffer, &reg, 1);
  memcpy(buffer + 1, bufp, len);
  err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, buffer, len + 1, true);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_FLUSH();
  return 0;
}

/**
 * @brief TWI initialization.
 */
static void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL1_PIN,
      .sda = ARDUINO_SDA1_PIN,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_LOW,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
  nrf_gpio_cfg(ARDUINO_SDA1_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
               NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(ARDUINO_SCL1_PIN, NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
               NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
}

/**
 * @brief Powering up capacitors and initializing pullup resistor.
 */
static void powerup_init(void) {
  nrf_gpio_pin_set(VDD_ENV_PIN);
  nrf_gpio_pin_set(R_PULLUP_PIN);
  // Power up and wait for voltage to rise
  nrf_gpio_cfg(VDD_ENV_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(R_PULLUP_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_delay_ms(4);
  ret_code_t err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for ANT stack initialization.
 */
static void softdevice_setup(void) {
  ret_code_t err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  ASSERT(nrf_sdh_is_enabled());

  err_code = nrf_sdh_ant_enable();
  APP_ERROR_CHECK(err_code);
}

static int get_feature_data(size_t offset, size_t length, float *out_ptr) {
  float features[length];
  feature_fifo_get(features, length);
  memcpy(out_ptr, features, length * sizeof(float));
  return 0;
}

EI_IMPULSE_ERROR run_classifier(signal_t *, ei_impulse_result_t *, bool);

/**
 * @brief Function for main application entry.
 */
int main(void) {
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  NRF_LOG_INFO("WBSLDC Start");

  powerup_init();
  NRF_LOG_INFO("Powering up device.");

  twi_init();
  NRF_LOG_INFO("TWI initialized.");

  softdevice_setup();
  NRF_LOG_INFO("Softdevice setup.");

  ant_master_setup();
  NRF_LOG_INFO("ANT message types setup.\r\n");

  /* Initialize inertial sensors (IMU) driver interface */
  uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H >> 1;
  lsm9ds1_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void *)&i2c_add_imu;
  NRF_LOG_INFO("IMU sensors initialized");

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_imu, &whoamI);
  if (whoamI != LSM9DS1_IMU_ID) {
    while (1) {
      /* manage device not found */
      NRF_LOG_INFO("Cannot find the LSM9DS1.********");
    }
  }
  NRF_LOG_INFO("Who am I register [IMU]: 0x%x", whoamI);

  /* Enable FIFO */
  lsm9ds1_fifo_mode_set(&dev_ctx_imu, LSM9DS1_STREAM_MODE);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_imu, PROPERTY_ENABLE);
  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_100);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_119Hz);
  NRF_LOG_INFO("IMU sensors configured");

  uint8_t imu_fifo_count = 0;

  while (true) {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_imu, &reg);

    if (reg.status_imu.xlda && reg.status_imu.gda) {
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_fifo_data_level_get(&dev_ctx_imu, &imu_fifo_count);
      NRF_LOG_INFO("Retrieving samples from IMU FIFO");
      for (uint8_t i = 0; i < imu_fifo_count; i++) {
        lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
        lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

        feature[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
        feature[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
        feature[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);
        feature[3] =
            lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        feature[4] =
            lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        feature[5] =
            lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
        feature_fifo_push(feature);
      }
      NRF_LOG_INFO("Retrieved %d samples from IMU FIFO", imu_fifo_count);

      if (feature_fifo_get_count() >= FIFO_BUFFER_SIZE) {
        NRF_LOG_INFO("FIFO full");
        uint16_t size = feature_fifo_get_count();
        float out[FIFO_BUFFER_SIZE];
        feature_fifo_get(out, FIFO_BUFFER_SIZE);
        NRF_LOG_INFO("FIFO emptied, %d", size);
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &get_feature_data;
        ei_impulse_result_t result;
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
        if (res != EI_IMPULSE_OK) {
          NRF_LOG_INFO("ERR: Failed to run classifier (%d)", res);
          continue;
        }

        uint8_t max_index = 0;
        for (uint8_t i = 1; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
          if (result.classification[i].value >
              result.classification[max_index].value) {
            max_index = i;
          }
        }
        NRF_LOG_INFO("Classification: %d %d%%", max_index,
                     (int)(result.classification[max_index].value * 100));
        if (result.classification[max_index].value > 0.9) {
          send_translation(max_index);
        }
      }
    }
  }
}

/** @} */
