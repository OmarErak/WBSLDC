/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Garmin Canada Inc. 2014
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 *    1) Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *
 *    2) Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *    3) Neither the name of Garmin nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior
 *       written permission.
 *
 * The following actions are prohibited:
 *
 *    1) Redistribution of source code containing the ANT+ Network
 *       Key. The ANT+ Network Key is available to ANT+ Adopters.
 *       Please refer to http://thisisant.com to become an ANT+
 *       Adopter and access the key.
 *
 *    2) Reverse engineering, decompilation, and/or disassembly of
 *       software provided in binary form under this license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 *
 */
#include "ant_master.h"

#include <stdint.h>

#include "ant_channel_config.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_sdh_ant.h"
#include "sdk_config.h"
#include "string.h"

// Channel configuration.
#define ANT_CUSTOM_TRANSMIT_POWER \
  0u /**< ANT Custom Transmit Power (Invalid/Not Used). */
#define BROADCAST_DATA_BUFFER_SIZE 8u  // < Size of the broadcast data buffer.
#define TRANSLATION_BUFFER_SIZE 64u    // < Size of the translation buffer.
#define PAGE_MAX_CHARACTER_NUM \
  9u  // < Maximum number of characters on a
      // page.

#define APP_ANT_OBSERVER_PRIO \
  1  // < Application's ANT observer priority. You shouldn't need to modify this
     // value.
#define STATUS_BIT_MASK 0x01
#define SEQ_NUM_MASK 0x3E
#define CHARACTER_BYTE_MASK 0x3F

// Static variables and buffers.
static uint8_t
    m_tx_buffer[BROADCAST_DATA_BUFFER_SIZE];  // < Primary data
                                              // (Broadcast/Acknowledged)
                                              // transmit buffer.

static uint8_t labels[4][9] = {{0x0F, 0x0B, 0x4, 0x0, 0x12, 0x4, 63, 63, 63},
                               {0x13, 0x7, 0x0, 0xD, 0xA, 0x14, 63, 63, 63},
                               {0x18, 0x4, 0x12, 63, 63, 63, 63, 63, 63}};

static uint8_t m_prev_index = 0xFF;

void ant_master_setup(void) {
  uint32_t err_code;

  ant_channel_config_t channel_config = {
      .channel_number = ANT_CHANNEL_NUM,
      .channel_type = CHANNEL_TYPE_MASTER,
      .ext_assign = 0x00,
      .rf_freq = RF_FREQ,
      .transmission_type = CHAN_ID_TRANS_TYPE,
      .device_type = CHAN_ID_DEV_TYPE,
      .device_number = (uint16_t)(NRF_FICR->DEVICEID[0]),
      .channel_period = CHAN_PERIOD,
      .network_number = ANT_NETWORK_NUM,
  };

  err_code = ant_channel_init(&channel_config);
  APP_ERROR_CHECK(err_code);

  // Set Tx Power
  err_code = sd_ant_channel_radio_tx_power_set(
      ANT_CHANNEL_NUM, RADIO_TX_POWER_LVL_3, ANT_CUSTOM_TRANSMIT_POWER);
  APP_ERROR_CHECK(err_code);

  // Open channel.
  err_code = sd_ant_channel_open(ANT_CHANNEL_NUM);
  APP_ERROR_CHECK(err_code);

  // Set default transmission buffer
  memset(m_tx_buffer, 0, BROADCAST_DATA_BUFFER_SIZE);

  // Configure the initial payload of the broadcast data
  err_code = sd_ant_broadcast_message_tx(
      ANT_CHANNEL_NUM, BROADCAST_DATA_BUFFER_SIZE, m_tx_buffer);
  APP_ERROR_CHECK(err_code);
}

void start_transmission(void) { m_tx_buffer[0] |= STATUS_BIT_MASK; }

void stop_transmission(void) { m_tx_buffer[0] &= ~STATUS_BIT_MASK; }

static void increment_sequence_number(void) {
  uint8_t seq_num = (m_tx_buffer[0] & SEQ_NUM_MASK) >> 1;
  m_tx_buffer[0] &= ~SEQ_NUM_MASK;
  m_tx_buffer[0] |= (((seq_num + 1) << 1) & SEQ_NUM_MASK);
}

static void populate_characters(uint8_t index) {
  m_tx_buffer[0] |= ((labels[index][0] & CHARACTER_BYTE_MASK) << 6) & 0xFF;
  m_tx_buffer[1] |= ((labels[index][0] & CHARACTER_BYTE_MASK) >> 2) & 0xFF;
  m_tx_buffer[1] |= ((labels[index][1] & CHARACTER_BYTE_MASK) << 4) & 0xFF;
  m_tx_buffer[2] |= ((labels[index][1] & CHARACTER_BYTE_MASK) >> 4) & 0xFF;
  m_tx_buffer[2] |= ((labels[index][2] & CHARACTER_BYTE_MASK) << 2) & 0xFF;
  m_tx_buffer[3] |= (labels[index][3] & CHARACTER_BYTE_MASK) & 0xFF;
  m_tx_buffer[3] |= ((labels[index][4] & CHARACTER_BYTE_MASK) << 6) & 0xFF;
  m_tx_buffer[4] |= ((labels[index][4] & CHARACTER_BYTE_MASK) >> 2) & 0xFF;
  m_tx_buffer[4] |= ((labels[index][5] & CHARACTER_BYTE_MASK) << 4) & 0xFF;
  m_tx_buffer[5] |= ((labels[index][5] & CHARACTER_BYTE_MASK) >> 4) & 0xFF;
  m_tx_buffer[5] |= ((labels[index][6] & CHARACTER_BYTE_MASK) << 2) & 0xFF;
  m_tx_buffer[6] |= labels[index][7] & CHARACTER_BYTE_MASK;
  m_tx_buffer[6] |= ((labels[index][8] & CHARACTER_BYTE_MASK) << 6) & 0xFF;
  m_tx_buffer[7] = 0;
  m_tx_buffer[7] |= ((labels[index][8] & CHARACTER_BYTE_MASK) >> 2) & 0xFF;
  m_tx_buffer[7] |= index << 4;
}

void send_translation(uint8_t index) {
  if (index == m_prev_index) {
    return;
  }

  memset(m_tx_buffer, 0, BROADCAST_DATA_BUFFER_SIZE);

  m_prev_index = index;

  start_transmission();
  increment_sequence_number();
  populate_characters(index);

  NRF_LOG_INFO("Sending translation: %d", index);

  uint32_t err_code = sd_ant_broadcast_message_tx(
      ANT_CHANNEL_NUM, BROADCAST_DATA_BUFFER_SIZE, m_tx_buffer);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
static void ant_evt_handler(ant_evt_t* p_ant_evt, void* p_context) {
  uint32_t err_code;

  switch (p_ant_evt->event) {
    // Send the next message according to the current state.
    case EVENT_TX:                     // Intentional fall through
    case EVENT_TRANSFER_TX_COMPLETED:  // Intentional fall through
    case EVENT_TRANSFER_TX_FAILED:
      // Rebroadcast the same data.
      err_code = sd_ant_broadcast_message_tx(
          ANT_CHANNEL_NUM, BROADCAST_DATA_BUFFER_SIZE, m_tx_buffer);
      APP_ERROR_CHECK(err_code);
    case TRANSFER_IN_PROGRESS:            // Intentional fall through
    case TRANSFER_SEQUENCE_NUMBER_ERROR:  // Intentional fall through
    case TRANSFER_IN_ERROR:               // Intentional fall through
    case TRANSFER_BUSY:
      break;

    default:
      break;  // No implementation needed
  }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler,
                     NULL);
