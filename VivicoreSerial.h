/*
  Copyright (c) 2021 VIVIWARE JAPAN, Inc. All right reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

  NeoHWSerial.h - Hardware serial library

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Created 2006 Nicholas Zambetti
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 31 October 2015 by SlashDev
*/

/**
 * @file VivicoreSerial.h
 * @brief Hardware serial library for VIVIWARE Cell Branch and Custom
 *
 * @example 01_servo.ino
 * @example 02_motor.ino
 * @example 03_joystick.ino
 * @example 04_button.ino
 * @example 05_rotator.ino
 * @example 06_slider.ino
 * @example 07_measure.ino
 * @example 08_motion.ino
 * @example 09_message_board.ino
 * @example 0A_color_picker.ino
 * @example 0B_led.ino
 */

#ifndef VIVICORESERIAL_H
#define VIVICORESERIAL_H

/** @cond */
#define BOARD_REV_FP1_EVT (1)
#define BOARD_REV_FP1_DVT (2)
#define BOARD_REV_FP1_PVT (3)
#define BOARD_REV         (BOARD_REV_FP1_DVT)

#define BOARD_TYPE_BRANCH (0)
#define BOARD_TYPE_CUSTOM (1)
#ifndef BOARD_TYPE
#  define BOARD_TYPE (BOARD_TYPE_BRANCH)
#endif

#if (BOARD_TYPE == BOARD_TYPE_CUSTOM)
#  define CORE_COMM_UART_PORT (1)
#  define SKIP_VERIFY_BRANCH_TYPE
#else
#  define CORE_COMM_UART_PORT (0)
#endif

#include <Arduino.h>
#include "CommunicationProtocol.h"
#include "VivicoreSerialDataCode.h"
#include "VivicoreSerialVersion.h"
#include "VivicoreSerialDebug.h"

// IO pin number definitions
#define PIN_DEBUG_LED (6)
#define PIN_EN_RX     (7)
#define PIN_EN_TX     (8)
#define PIN_EN_PWR    (A3)

#define NUM_SERIAL_BYTES (10)
#define NUM_BRTYPE_BYTES (4)

#define NUM_MAX_TEMP_BUFF (30) // maximum byte of tempBuffer
#define NUM_MAX_READ_BUFF (64) // To be removed: maximum byte of RingBuffer<unsigned char>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#if (RAMEND < 1000)
#  define SERIAL_BUFFER_SIZE (1UL << 4)
#else
#  define SERIAL_BUFFER_SIZE (1UL << 6)
#endif

#define RING_PKT_BUFFER_SIZE (10)

template <typename T, size_t N> size_t countof(const T (&)[N]) {
  return N;
}

template <typename T, size_t SIZE> struct RingBuffer {
  T                     buffer[SIZE] = {};
  volatile unsigned int head;
  volatile unsigned int tail;

  RingBuffer(const unsigned int HEAD, const unsigned int TAIL) : head(HEAD), tail(TAIL) {}
};

enum dataType_t {
  DATA_TYPE_NONE = 0,
  DATA_TYPE_RAW,
  DATA_TYPE_DCDT,
};

struct pkt_payload_t {
  uint8_t    data[NUM_MAX_UART_PKT_BODY_DATA];
  uint16_t   size;
  dataType_t type;
};
/** @endcond */

/**
 * @brief This is data buffer structure to read scaler data from core
 */
struct ScalerData_t {
  uint8_t dc_n;    /**< DC number of the scaler data */
  int32_t data;    /**< Scaler data of the DC number which is read from core */
  bool    success; /**< true is no error, and another is some error */
};

/**
 * @brief This is data buffer structure to read raw data from core
 */
struct RawData_t {
  uint8_t data[NUM_MAX_UART_PKT_BODY_DATA]; /**< Raw data which is read from core */
  size_t  data_len;                         /**< Actual data length in data buffer */
  bool    success;                          /**< true is no error, and another is some error */
};

/**
 * @brief This is data buffer structure to get the number of available data for @ref VivicoreSerial::available
 */
struct AvailableNum_t {
  uint8_t raw;    /**< Number of available raw data which should be 1 in a packet */
  uint8_t scaler; /**< Number of available scaler data */
};

/**
 * @brief This class is for branch to talk with core
 */
class VivicoreSerial {
public:
  /** @cond */
  VivicoreSerial(volatile uint8_t *ubrrh, // USART baudrate register high
                 volatile uint8_t *ubrrl, // USART baudrate register low
                 volatile uint8_t *ucsra, // USART Control and Status Registers
                 volatile uint8_t *ucsrb, // USART Control and Status Registers
                 volatile uint8_t *ucsrc, // USART Control and Status Registers
                 volatile uint8_t *udr,   // USART Data Register
                 uint8_t           rxen,  // Receiver Enable bit on UCSR0B, UCSR1B
                 uint8_t           txen,  // Transmitter Enable bit on UCSR0B, UCSR1B
                 uint8_t           rxcie, // RX Complete Interrupt Enable bit on UCSR0B, UCSR1B
                 uint8_t           udrie, // Data Register Empty Interrupt Enable bit on UCSR0A, UCSR1A
                 uint8_t           u2x,   // Double the USART Transmission Speed bit on UCSR0A, UCSR1A
                 uint8_t           txc,   // USART Transmit Complete bit on UCSR0A, UCSR1A
                 uint8_t           dor,   // Data OverRun bit on UCSR0A, UCSR1A
                 uint8_t           upe,   // USART Parity Error bit on UCSR0A, UCSR1A
                 uint8_t           fe);             // Frame Error bit on UCSR0A, UCSR1A
  virtual ~VivicoreSerial(void);
  /** @endcond */

  /**
   * This API initializes this library instance and store configuration to communicate with core. The instance can be
   * configured by arguments of this API. This API returns true or false which is returned by
   * @ref DataCodeTranslator::init.
   *
   * @param [in] branch_type Branch type value
   * @param [in] user_version Version number for user program
   * @param [in] dc_info Pointer to buffer of DC info array
   * @param [in] dc_num Number of DC info array
   * @param [in] min_lib_buildno (Optional) Minimum required library build number which is defined as
   * @ref LIBRARY_VER_BUILD_NO
   * @return true if no error, or another in error case.
   */
  bool begin(const uint32_t branch_type, const uint16_t user_version, const dcInfo_t *dc_info, const uint8_t dc_num,
             const uint16_t min_lib_buildno = 0);

  /**
   * Before using @ref read or @ref readRaw, the number of available data should be checked with this API. This API
   * returns @ref AvailableNum_t as the following.
   *
   * - If @ref AvailableNum_t::scaler is greater than 0:
   *   - This API has stored data received from core and decoded by @ref DataCodeTranslator::decode
   *   - @ref AvailableNum_t::scaler is the number of DC decoded
   *     (e.g. The number is 2 if data for DC 1 and 2 is received)
   *   - The decoded data may have some errors which are handled in @ref read
   * - If @ref AvailableNum_t::raw is 1:
   *   - This API has stored a raw data packet received from core
   *
   * @return Number of scaler and raw data received from core. 0 is no data available.
   */
  AvailableNum_t available(void);

  /**
   * This API returns data including decoded DC number, scaler value, and success information. Before calling this API,
   * it is necessary to check if there is available data with @ref available. This API returns false in
   * @ref ScalerData_t::success and does nothing in any case of the following conditions.
   *
   * - In case that @ref available returns 0 on @ref AvailableNum_t::scaler
   *
   * This API returns false in @ref ScalerData_t::success but also returns the stored data in any case of the following
   * conditions.
   *
   * - data_scaler is out of range between DC min and DC max specified on DC info by @ref begin
   * - dc_n is for @ref DC_NATURE_OUT specified on DC info by @ref begin
   * - dc_n is for @ref DC_TYPE_BINARY specified on DC info by @ref begin
   *
   * @return Read data from core. @ref ScalerData_t::success is true if no error. false in another case.
   */
  ScalerData_t read(void);

  /**
   * This API stores raw data received from core. The raw data is equivalent to the received packet body. Before calling
   * this API, it is necessary to check if there is available data with @ref available. This API returns false in
   * @ref RawData_t::success and does nothing in any case of the following conditions.
   *
   * - In case that @ref available returns 0 on @ref AvailableNum_t::raw
   *
   * @return Read data from core. @ref RawData_t::success is true if no error. false in another case.
   */
  RawData_t readRaw(void);

  /**
   * This API stores scaler data to be sent to core corresponding to DC number. It is necessary to call @ref flush to
   * send the stored data to core. The stored value specified by DC number can be overwritten if this API was called
   * multiple times. This API returns error and does not store the specified data in any case of the following
   * conditions.
   *
   * - dc_n is out of range between 1 to the maximum value @ref NUM_MAX_DC in specification
   *
   * @param [in] dc_n DC number corresponding to data_scaler
   * @param [in] data_scaler Scaler data to be sent to core
   * @return true if no error, or another in error case.
   *
   * @warning This API should not be used with @ref writeRaw in user probram on a *.ino file. If you do that, the
   * program may behave unintentionally.
   */
  bool write(const uint8_t dc_n, const int32_t data_scaler);

  /**
   * This API stores raw data to be sent to core. The raw data is equivalent to the sending packet body. It is
   * necessary to call @ref flush to send the stored data to core. The stored data can be overwritten if this API was
   * called multiple times. This API returns error but stores data truncated with the internal buffer size in any case
   * of the following conditions.
   *
   * - data_len is over @ref NUM_MAX_UART_PKT_BODY_DATA
   *
   * @param [in] data Pointer to buffer of raw data to be sent to core
   * @param [in] data_len Size of buffer of raw data to be sent to core
   * @param [in] data_type Type of raw data to be sent to core
   * @return true if no error, or another in error case.
   *
   * @warning This API now supports @ref DATA_TYPE_DCDT only.
   * @warning This API should not be used with @ref write in user probram on a *.ino file. If you do that, the program
   * may behave unintentionally.
   */
  bool writeRaw(const uint8_t *data, const size_t data_len, const dataType_t data_type = DATA_TYPE_DCDT);

  /**
   * This API encodes the data stored by @ref write. This API sends the encoded data which is succeeded to be encoded or
   * raw data stored by @ref writeRaw, and waits to finish sending. This API does nothing and returns if no data to
   * send.
   *
   * @return true if no error, or another in error case of @ref DataCodeTranslator::encode.
   */
  bool flush(void);

  /**
   * This API terminates talking with core as the following sequence.
   * 1. Wait for transmission of outgoing data
   * 2. Disable UART to talk with core
   * 3. Drop all received data
   */
  void end(void);

  /**
   * @cond
   * Below members are NOT for user but for interrupt handler to access VivicoreSerial internal state
   */
  bool isInFatalError(void);

  BranchCommand_t    parseCommand(const uint8_t c);
  BranchCommandRes_t processCommand(const BranchCommand_t cmd);
  bool               sendResponse(const BranchCommand_t bcmd, const BranchCommandRes_t res_type);

  void clearTransmitting(void);
  void setSyncBreakReceived(void);
  bool setOverrideIni(const uint8_t dc_idx, const int16_t val, const dcInfo_t *dc_info, const uint8_t dc_num);

  volatile uint8_t *const _ubrrh; // USART baudrate register high
  volatile uint8_t *const _ubrrl; // USART baudrate register low
  volatile uint8_t *const _ucsra; // USART Control and Status Registers
  volatile uint8_t *const _ucsrb; // USART Control and Status Registers
  volatile uint8_t *const _ucsrc; // USART Control and Status Registers
  volatile uint8_t *const _udr;   // USART Data Register
  const uint8_t           _rxen;  // Receiver Enable bit on UCSR0B, UCSR1B
  const uint8_t           _txen;  // Transmitter Enable bit on UCSR0B, UCSR1B
  const uint8_t           _rxcie; // RX Complete Interrupt Enable bit on UCSR0B, UCSR1B
  const uint8_t           _udrie; // Data Register Empty Interrupt Enable bit on UCSR0A, UCSR1A
  const uint8_t           _u2x;   // Double the USART Transmission Speed bit on UCSR0A, UCSR1A
  const uint8_t           _txc;   // USART Transmit Complete bit on UCSR0A, UCSR1A
  const uint8_t           _dor;   // Data OverRun bit on UCSR0A, UCSR1A
  const uint8_t           _upe;   // USART Parity Error bit on UCSR0A, UCSR1A
  const uint8_t           _fe;    // Frame Error bit on UCSR0A, UCSR1A

  RingBuffer<pkt_payload_t, RING_PKT_BUFFER_SIZE> *_rx_buffer = nullptr;
  RingBuffer<unsigned char, SERIAL_BUFFER_SIZE> *  _tx_buffer = nullptr;

  bool _is_passthru_mode = false;
  /** @endcond */

private:
  struct scalerDataW_t {
    int16_t body[NUM_MAX_DC];
    bool    is_set[NUM_MAX_DC];
  };

  struct scalerDataR_t {
    int16_t body[NUM_MAX_DC];
    uint8_t dc_nums[NUM_MAX_DC];
    uint8_t dc_nums_count;
  };

  struct overrideIni_t {
    bool    set;
    int16_t data_ini;
  };

  const uint8_t _signature[3];
  const uint8_t _serial_number[NUM_SERIAL_BYTES];

  uint8_t  _data_by_user[NUM_MAX_UART_PKT]  = {};
  uint8_t  _data_response[NUM_MAX_UART_PKT] = {};
  uint8_t *_cmd_params                      = nullptr;
  uint8_t  _cmd_params_len                  = 0;
  uint8_t  _data_len_by_user                = 0;

  scalerDataR_t _scaler_data_by_core = {};
  scalerDataW_t _scaler_data_by_user = {};
  data_pkt      _raw_data_by_core    = {};
  data_pkt      _raw_data_by_user    = {};

  DataCodeTranslator *_translator = nullptr;

  const dcInfo_t *_dc_info                  = nullptr;
  uint8_t         _dc_num                   = 0;
  overrideIni_t   _override_ini[NUM_MAX_DC] = {};
  bool            _dominate_led             = false;

  volatile bool _is_dcdt_ok             = false;
  volatile bool _send_flag              = false; // Written data is available for transmission if true
  bool          _is_read_polling        = false;
  bool          _in_transmitting        = false; // UART transmission is on-going if true
  bool          _is_sync_break_received = false; // sync break is issued if true

  uint8_t                  _my_branch_id      = 0;
  uint32_t                 _my_branch_type    = 0;
  uint16_t                 _user_fw_ver       = 0;
  uint16_t                 _min_lib_buildno   = 0;
  bool                     _fatal_mode        = false;
  BranchCommandParamFind_t _find_branch_mode  = BCMDPARAM_FIND_BLINK_OFF;
  uint32_t                 _saved_branch_type = 0;

  VivicoreSerial(const VivicoreSerial &);
  VivicoreSerial &operator=(const VivicoreSerial &);

  void   init(void);
  bool   pullFromRxRingBuff(data_pkt *raw_data);
  void   pushToRxRingBuff(const uint8_t *buffer, const uint8_t length, const dataType_t data_type);
  size_t pushToTxRingBuff(const uint8_t c);
  void   pushToTxRingBuffAndTransmit(const uint8_t *buffer, const uint8_t datalen);

  void setTransmitting(void);
  bool isTransmitting(void);
  bool isSyncBreakReceived(void);

  void setBaud(unsigned long baud);
  void setSyncBreak(void);

  void controlLedBlink(const bool *timing_table, const uint8_t max_count);
  void controlLedOnOff(const bool stop_blink);
  void manageLedState(void);

  uint8_t getCRC8(const uint8_t *buff, const size_t size);
};

extern VivicoreSerial Vivicore; /**< VivicoreSerial singleton instance */

#endif // VIVICORESERIAL_H
