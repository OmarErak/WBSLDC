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
#include "app_fifo.h"
#include "app_util_platform.h"
#include "boards.h"
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
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
#define FIFO_BUFFER_SIZE (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * 2) / 6

/* Private variables ---------------------------------------------------------*/
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float imu_data[6];
static uint16_t fifo_data_index = 0;
static app_fifo_t xl_x_fifo;
static app_fifo_t xl_y_fifo;
static app_fifo_t xl_z_fifo;
static app_fifo_t g_x_fifo;
static app_fifo_t g_y_fifo;
static app_fifo_t g_z_fifo;
static uint8_t xl_x_fifo_buffer[512];  // Must be power of 2
static uint8_t xl_y_fifo_buffer[512];  // Must be power of 2
static uint8_t xl_z_fifo_buffer[512];  // Must be power of 2
static uint8_t g_x_fifo_buffer[512];   // Must be power of 2
static uint8_t g_y_fifo_buffer[512];   // Must be power of 2
static uint8_t g_z_fifo_buffer[512];   // Must be power of 2
const uint8_t sample_size = 2;

static uint8_t ant_buffer[8];

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
  NRF_LOG_FLUSH();
  return 0;
}

/**
 * @brief TWI initialization.
 */
static void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = NRF_GPIO_PIN_MAP(0, 26),
      .sda = NRF_GPIO_PIN_MAP(0, 27),
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_LOW,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
  // nrf_gpio_cfg(ARDUINO_SDA1_PIN, NRF_GPIO_PIN_DIR_INPUT,
  //              NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
  //              NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
  // nrf_gpio_cfg(ARDUINO_SCL1_PIN, NRF_GPIO_PIN_DIR_INPUT,
  //              NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
  //              NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
}

/**
 * @brief Powering up capacitors and initializing pullup resistor.
 */
static void powerup_init(void) {
  // nrf_gpio_pin_set(VDD_ENV_PIN);
  // nrf_gpio_pin_set(R_PULLUP_PIN);
  // // Power up and wait for voltage to rise
  // nrf_gpio_cfg(VDD_ENV_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
  //              NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
  //              NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  // nrf_gpio_cfg(R_PULLUP_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
  //              NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
  //              NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  // nrf_delay_ms(4);
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

static int16_t get_sample(uint8_t hb, uint8_t lb) {
  uint16_t sample = (hb << 8) | lb;
  if (sample & 0x8000) {
    sample &= 0x7FFF;
    return -sample;
  }
  return (int16_t)sample;
}

static int get_feature_data(size_t offset, size_t length, float *out_ptr) {
  uint16_t buffer_length = (length * 2) / 6;
  uint8_t xl_x_buffer[buffer_length];
  uint8_t xl_y_buffer[buffer_length];
  uint8_t xl_z_buffer[buffer_length];
  uint8_t g_x_buffer[buffer_length];
  uint8_t g_y_buffer[buffer_length];
  uint8_t g_z_buffer[buffer_length];
  uint32_t err_code = app_fifo_read(&xl_x_fifo, xl_x_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);
  err_code = app_fifo_read(&xl_y_fifo, xl_y_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);
  err_code = app_fifo_read(&xl_z_fifo, xl_z_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);
  err_code = app_fifo_read(&g_x_fifo, g_x_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);
  err_code = app_fifo_read(&g_y_fifo, g_y_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);
  err_code = app_fifo_read(&g_z_fifo, g_z_buffer, &buffer_length);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Retrieving %d samples from FIFO", length);

  for (uint16_t i = 0; i < length; i += 6) {
    uint16_t index = i / 6;
    out_ptr[i] = lsm9ds1_from_fs4g_to_mg(
        get_sample(xl_x_buffer[index + 1], xl_x_buffer[index]));
    out_ptr[i + 2] = lsm9ds1_from_fs4g_to_mg(
        get_sample(xl_y_buffer[index + 1], xl_y_buffer[index]));
    out_ptr[i + 4] = lsm9ds1_from_fs4g_to_mg(
        get_sample(xl_z_buffer[index + 1], xl_z_buffer[index]));
    out_ptr[i + 1] = lsm9ds1_from_fs2000dps_to_mdps(
        get_sample(g_x_buffer[index + 1], g_x_buffer[index]));
    out_ptr[i + 3] = lsm9ds1_from_fs2000dps_to_mdps(
        get_sample(g_y_buffer[index + 1], g_y_buffer[index]));
    out_ptr[i + 5] = lsm9ds1_from_fs2000dps_to_mdps(
        get_sample(g_z_buffer[index + 1], g_z_buffer[index]));
  }

  NRF_LOG_INFO("Retrieved %d samples from FIFO", length);
}

EI_IMPULSE_ERROR run_classifier(signal_t *, ei_impulse_result_t *, bool);

// void uart_error_handle(app_uart_evt_t * p_event)
// {
//   if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//   {
//     APP_ERROR_HANDLER(p_event->data.error_communication);
//   }
//   else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//   {
//     APP_ERROR_HANDLER(p_event->data.error_code);
//   }
// }

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

  // ant_message_types_master_setup();
  // NRF_LOG_INFO("ANT message types setup.");

  nrf_log_backend_rtt_init();

  uint32_t err_code =
      app_fifo_init(&xl_x_fifo, xl_x_fifo_buffer, sizeof(xl_x_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  err_code =
      app_fifo_init(&xl_y_fifo, xl_y_fifo_buffer, sizeof(xl_y_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  err_code =
      app_fifo_init(&xl_z_fifo, xl_z_fifo_buffer, sizeof(xl_z_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  err_code = app_fifo_init(&g_x_fifo, g_x_fifo_buffer, sizeof(g_x_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  err_code = app_fifo_init(&g_y_fifo, g_y_fifo_buffer, sizeof(g_y_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  err_code = app_fifo_init(&g_z_fifo, g_z_fifo_buffer, sizeof(g_z_fifo_buffer));
  APP_ERROR_CHECK(err_code);

  // const app_uart_comm_params_t comm_params = {
  //   RX_PIN_NUMBER,
  //   TX_PIN_NUMBER,
  //   RTS_PIN_NUMBER,
  //   CTS_PIN_NUMBER,
  //   APP_UART_FLOW_CONTROL_DISABLED,
  //   false,
  //   NRF_UARTE_BAUDRATE_115200
  // };

  // uint32_t err_code;
  // APP_UART_FIFO_INIT(&comm_params,
  //                   256,
  //                   256,
  //                   uart_error_handle,
  //                   APP_IRQ_PRIORITY_LOWEST,
  //                   err_code);

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

  uint16_t fifo_count = 0;
  uint8_t fifo_clear_buffer[sample_size * 6];
  uint8_t imu_fifo_count = 0;
  float out[1200];

  while (true) {
    // uint8_t cr = 0xFC;
    // while (app_uart_put(cr) != NRF_SUCCESS);

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

        err_code = app_fifo_write(
            &xl_x_fifo, (uint8_t *)(&(data_raw_acceleration.i16bit[0])),
            &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_write(
            &xl_y_fifo, (uint8_t *)(&(data_raw_acceleration.i16bit[1])),
            &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_write(
            &xl_z_fifo, (uint8_t *)(&(data_raw_acceleration.i16bit[2])),
            &sample_size);
        APP_ERROR_CHECK(err_code);

        err_code = app_fifo_write(
            &g_x_fifo, (uint8_t *)(&(data_raw_angular_rate.i16bit[0])),
            &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_write(
            &g_y_fifo, (uint8_t *)(&(data_raw_angular_rate.i16bit[1])),
            &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_write(
            &g_z_fifo, (uint8_t *)(&(data_raw_angular_rate.i16bit[2])),
            &sample_size);
        APP_ERROR_CHECK(err_code);

        // NRF_LOG_INFO("Acceleration: %x%x, %x%x, %x%x",
        // data_raw_acceleration.u8bit[1], data_raw_acceleration.u8bit[0],
        // data_raw_acceleration.u8bit[3], data_raw_acceleration.u8bit[2],
        // data_raw_acceleration.u8bit[5], data_raw_acceleration.u8bit[4]);
      }
      NRF_LOG_INFO("Retrieved %d samples from IMU FIFO", imu_fifo_count);

      err_code = app_fifo_read(&xl_x_fifo, NULL, &fifo_count);
      APP_ERROR_CHECK(err_code);

      if (fifo_count >= FIFO_BUFFER_SIZE) {
        err_code = app_fifo_read(&xl_x_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_read(&xl_y_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_read(&xl_z_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_read(&g_x_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_read(&g_y_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);
        err_code = app_fifo_read(&g_z_fifo, &fifo_clear_buffer, &sample_size);
        APP_ERROR_CHECK(err_code);

        get_feature_data(0, 1200, out);

        // signal_t signal;
        // signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        // signal.get_data = &get_feature_data;
        // ei_impulse_result_t result;
        // EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

        // NRF_LOG_INFO("Classified as: %s (%d)",
        // result.classification[0].label, (int)result.classification[0].value *
        // 100); NRF_LOG_INFO("Other possibilities: %s (%d), %s (%d), %s (%d)",
        //   result.classification[1].label, (int)result.classification[1].value
        //   * 100, result.classification[2].label,
        //   (int)result.classification[2].value * 100,
        //   result.classification[3].label, (int)result.classification[3].value
        //   * 100);

        // memset(ant_buffer, 0, sizeof(ant_buffer));
        // ant_buffer[4] = (uint8_t) result.classification[0].value * 100;
        // ant_buffer[5] = (uint8_t) result.classification[1].value * 100;
        // ant_buffer[6] = (uint8_t) result.classification[2].value * 100;
        // ant_buffer[7] = (uint8_t) result.classification[3].value * 100;
      }
    }
    // ant_buffer[1] = (uint8_t) (fifo_count & 0xFF0000) >> 16;
    // ant_buffer[2] = (uint8_t) (fifo_count & 0xFF00) >> 8;
    // ant_buffer[3] = (uint8_t) (fifo_count & 0xFF);
    // send_broadcast(ant_buffer, 8);

    // nrf_pwr_mgmt_run();
  }
}

/** @} */
