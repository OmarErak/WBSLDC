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

#include <stdio.h>

#include "ant_master.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
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

/* Private variables ---------------------------------------------------------*/
static lsm9ds1_id_t whoamI;
char tx_buffer[100];
// static uint8_t rst;
static lsm9ds1_status_t reg;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float acceleration_mg[3];
static float angular_rate_mdps[3];

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
  if (err_code == NRF_SUCCESS) {
    NRF_LOG_INFO("Device Address and Register Address and Data sent");
  }
  NRF_LOG_FLUSH();
  return 0;
}

/**
 * @brief TWI initialization.
 */
void twi_init(void) {
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
void powerup_init(void) {
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

/**
 * @brief Function for main application entry.
 */
int main(void) {
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  powerup_init();
  printf("Powering up device.\r\n");

  twi_init();
  printf("\r\nTWI initialized.\r\n");

  softdevice_setup();
  printf("Softdevice setup.\r\n");

  ant_message_types_master_setup();
  printf("ANT message types setup.\r\n");

  /* Initialize inertial sensors (IMU) driver interface */
  uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H >> 1;
  lsm9ds1_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void *)&i2c_add_imu;
  printf("IMU sensors initialized\r\n");

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_imu, &whoamI);
  if (whoamI != LSM9DS1_IMU_ID) {
    while (1) {
      /* manage device not found */
      printf("\r\nCannot find the LSM9DS1.********\r\n");
    }
  }
  printf("Who am I register [IMU]: 0x%x\r\n", whoamI);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_imu, PROPERTY_ENABLE);

  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);

  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);

  while (true) {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_imu, &reg);

    if (reg.status_imu.xlda && reg.status_imu.gda) {
      /* Read imu data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

      acceleration_mg[0] =
          lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
          lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
          lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

      angular_rate_mdps[0] =
          lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] =
          lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] =
          lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      uint8_t ant_buffer[8] = {0};
      ant_buffer[0] = (int)acceleration_mg[0];
      ant_buffer[1] = (int)acceleration_mg[1];
      ant_buffer[2] = (int)acceleration_mg[2];
      ant_buffer[3] = (int)angular_rate_mdps[0];
      ant_buffer[4] = (int)angular_rate_mdps[1];
      ant_buffer[5] = (int)angular_rate_mdps[2];
      send_broadcast(ant_buffer, 8);

      nrf_pwr_mgmt_run();
    }
  }
}

/** @} */
