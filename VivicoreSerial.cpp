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

  NeoHWSerial.cpp - Hardware serial library

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
 * @cond
 * @file VivicoreSerial.cpp
 * @brief Hardware serial library for VIVIWARE Cell Branch and Custom
 * @endcond
 */

#include <wiring_private.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include "VivicoreSerial.h"

// Product spec
#define NUM_MAX_SLAVE (5)
#define SLOW_BAUD     ((unsigned long)(UART_PROTOCOL_BAUDRATE * 8 / 9)) // Used for sync_break = 1000*8/(1000/baudrate*9)

#define DATA_PAYLOAD_HEAD       (4)
#define STARTUP_LED_ON_DURATION (1500)
#define LED_BLINK_INTERVAL      (80)

// Macro functions
#define setIndicatorLed(led_state) \
  { digitalWrite(PIN_VIVIWARE_DEBUG_LED, !(led_state)); }

enum StateStoreChar_t {
  STATE_STORE_CHAR_NONE = 0x00,
  STATE_STORE_CHAR_STX, // ready for incoming traffic
  STATE_STORE_CHAR_LENGTH,
  STATE_STORE_CHAR_CMD,
  STATE_STORE_CHAR_BRANCH_ID,
  STATE_STORE_CHAR_DATA_BODY_AND_CRC,
};

#if !defined(IN_TEST)
#  if (CORE_COMM_UART_PORT == 0)
VivicoreSerial Vivicore(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0, TXC0,
                        DOR0, UPE0, FE0);
#  elif (CORE_COMM_UART_PORT == 1)
VivicoreSerial Vivicore(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1, RXEN1, TXEN1, RXCIE1, UDRIE1, U2X1, TXC1,
                        DOR1, UPE1, FE1);
#  else
#    error Not supported!
#  endif
#endif

static bool infinite_wait_in_reset  = true; // Infinitely wait for reset by WD after received RESET CMD if true
static void (*sleep_in_flush)(void) = nullptr;
#if defined(IN_TEST)
static VivicoreSerial *active_object = NULL;

void activateVivicoreObject(VivicoreSerial *obj) {
  active_object = obj;
}

void configureVivicoreObject(void (*sleep_in_flush_)(void), bool infinite_wait_in_reset_) {
  sleep_in_flush         = sleep_in_flush_;
  infinite_wait_in_reset = infinite_wait_in_reset_;
}
#else
static VivicoreSerial *active_object = &Vivicore;
#endif

// Static Utility Function
namespace {
void dump(const uint8_t *buff, const uint8_t buffLen) {
  DebugStringPrint0("buffLen=");
  DebugPlainPrintln0(buffLen);
  for (int i = 0; i < buffLen; i++) {
    DebugHexPrint0(buff[i]);
  }
}
}; // namespace

// Class members implementation
void VivicoreSerial::setTransmitting(void) {
  _in_transmitting = true;
}

void VivicoreSerial::clearTransmitting(void) {
  _in_transmitting = false;
}

bool VivicoreSerial::isTransmitting(void) {
  return _in_transmitting;
}

void VivicoreSerial::setSyncBreakReceived(void) {
  _is_sync_break_received = true;
}

bool VivicoreSerial::isSyncBreakReceived(void) {
  const bool is_sync_break_received_ = _is_sync_break_received;
  _is_sync_break_received            = false;
  return is_sync_break_received_;
}

BranchCommand_t VivicoreSerial::parseCommand(const uint8_t c) {
  static uint8_t          data_rx_raw[NUM_MAX_TEMP_BUFF]; // full pkt receive buffer without CRC8
  static uint8_t          length_wo_header = 0;
  static uint8_t          offset           = 0;
  static bool             reset_previously = false;
  static uint8_t          proc_cmd         = (uint8_t)BCMD_INVALID;
  static bool             is_my_packet     = false;
  static StateStoreChar_t now_state        = STATE_STORE_CHAR_STX;
  StateStoreChar_t        next_state       = now_state;
  uint8_t                 rcv_branch_id;
  bool                    can_be_parsed = false;

  // DebugHexPrint0(c);
  // DebugStringPrintln0("");
  // DebugStackPointerPrint();
  // DebugHexPrint1(c);
  // DebugStringPrint1("\n");

  if (isSyncBreakReceived()) {
    reset_previously = true;
    offset           = 0;
    proc_cmd         = (uint8_t)BCMD_INVALID;
    now_state        = STATE_STORE_CHAR_STX;
    return BCMD_INVALID; // assume c is garbage data
  }
  switch (now_state) {
  case STATE_STORE_CHAR_STX:
    is_my_packet     = true; // assume this pkt is mine
    length_wo_header = 0;

    // If break happened in previous frame and current byte is STX, then make sequencer proceed
    if (reset_previously && c == STX) {
      data_rx_raw[offset++] = c;
      next_state            = STATE_STORE_CHAR_LENGTH;
    } else {
      offset     = 0;
      next_state = STATE_STORE_CHAR_STX;
    }
    reset_previously = false;
    break;

  case STATE_STORE_CHAR_LENGTH:
    data_rx_raw[offset++] = c;
    length_wo_header      = c;

    if (offset < sizeof(data_rx_raw)) {
      next_state = STATE_STORE_CHAR_CMD;
    } else {
      offset     = 0;
      next_state = STATE_STORE_CHAR_STX;
    }
    break;

  case STATE_STORE_CHAR_CMD:
    data_rx_raw[offset++] = c;
    proc_cmd              = c;

    if (offset < sizeof(data_rx_raw)) {
      next_state = STATE_STORE_CHAR_BRANCH_ID;
    } else {
      offset     = 0;
      next_state = STATE_STORE_CHAR_STX;
    }
    break;

  case STATE_STORE_CHAR_BRANCH_ID:
    data_rx_raw[offset++] = c;
    rcv_branch_id         = c;

    if (offset < sizeof(data_rx_raw) && rcv_branch_id <= NUM_MAX_SLAVE) {
      next_state = STATE_STORE_CHAR_DATA_BODY_AND_CRC;
      if (((uint8_t)BCMD_DISCOVERY == proc_cmd) ||
          ((uint8_t)BCMD_RESET == proc_cmd && BRANCH_ID_ROOT == _my_branch_id) || (rcv_branch_id == _my_branch_id)) {
        // Treat this packet for mine
      } else {
        // This packet is not mine
        is_my_packet = false;
      }
    } else {
      // Invalid branch id
      offset     = 0;
      next_state = STATE_STORE_CHAR_STX;
    }
    break;

  case STATE_STORE_CHAR_DATA_BODY_AND_CRC:
    data_rx_raw[offset++] = c;

    if (offset < sizeof(UartPktHdr_t) + length_wo_header && offset < sizeof(data_rx_raw)) {
      if ((uint8_t)BCMD_DISCOVERY == proc_cmd && offset - 1 == DATA_PAYLOAD_HEAD) {
        // _my_branch_id is already set and next Address is not me
        if (_my_branch_id && data_rx_raw[DATA_PAYLOAD_HEAD] != _my_branch_id) {
          is_my_packet = false; // this pkt is not mine
        }
      }
    } else {
      next_state = STATE_STORE_CHAR_STX;
      if (c == getCRC8(data_rx_raw, offset - 1)) {
        if (is_my_packet) {
          can_be_parsed   = true;
          _cmd_params     = &data_rx_raw[DATA_PAYLOAD_HEAD]; // discard header, BCMD, and target branch ID
          _cmd_params_len = length_wo_header - 1 - 1 - 1;    // minus BCMD, target branch ID, and CRC8

#if defined(SERIAL_DEBUG_COBI_ENABLED)
          DebugPrintCOBI("COBI:");
          for (uint8_t i = 0; i < offset; i++) {
            DebugHexPrintCOBI(data_rx_raw[i]);
          }
          DebugPrintCOBIln("");
#endif // SERIAL_DEBUG_COBI_ENABLED

          manageLedState();
        }
        // DebugHexPrint1(proc_cmd);
        // DebugStringPrint1("\n");
      } else {
        DebugStringPrint0("DatErr=");
        dump(data_rx_raw, offset);
        DebugStringPrint0("\n");
        DebugStringPrint0("UCSRxA=");
        DebugHexPrint0(*_ucsra);
        DebugStringPrint0("\n");
      }
      offset = 0;
    }
    break;
  default:
    break;
  }

  if (next_state != now_state) {
    now_state = next_state;
  }
  // int i = (unsigned int)(_rx_buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // // if we should be storing the received character into the location
  // // just before the tail (meaning that the head would advance to the
  // // current location of the tail), we're about to overflow the buffer
  // // and so we don't write the character or advance the head.
  // if (i != _rx_buffer->tail) {
  //   _rx_buffer->buffer[ _rx_buffer->head ] = c;
  //   _rx_buffer->head = i;
  // }

  DebugGPIOLow(PORTD, 2);  // debug D2 PD2
  DebugGPIOHigh(PORTD, 2); // debug D2 PD2

  return can_be_parsed ? (BranchCommand_t)proc_cmd : BCMD_INVALID;
}

BranchCommandRes_t VivicoreSerial::processCommand(const BranchCommand_t cmd) {
  BranchCommandRes_t ret = BCMDRES_NOTHING;

  if (_translator->hasFatalError()) {
    DebugStringPrintln0("F: Fatal error in DC info configuration!");
    return ret;
  }

  switch (cmd) {
  // Should send response data against the below commands
  case BCMD_RESET:
    // Reset command
    if (BRANCH_ID_ROOT == _my_branch_id && 0x00 == _cmd_params[0] && 0x00 == _cmd_params[1]) {
      DebugStringPrintln0("Reboot!");
      setIndicatorLed(true);
      DebugFlush();
      wdt_disable();
      wdt_enable(WDTO_15MS);
      while (infinite_wait_in_reset) {
        // Wait for rebooting by WDT
      };
    }
    break;
  case BCMD_DISCOVERY:
    if (_my_branch_id == 0x00) {
      DebugStringPrintln0("Only if 1st DISCOVERY");
      _my_branch_id = _cmd_params[0];
    } else if (_my_branch_id == _cmd_params[0]) {
      DebugStringPrintln0("Reset to STEP_DISCOVERY");
      _is_read_polling = false;
      _is_dcdt_ok      = false;
      // _send_flag = false;  // Keep TX data set b/w 1st and another discovery
    }
    ret = BCMDRES_DISCOVERY_ACK;
    break;
  case BCMD_READ:
    switch (_cmd_params[0]) {
    case BCMDPARAM_READ_ENCODED_DATA:
      if (!_is_read_polling) {
        // First BCMDPARAM_READ_ENCODED_DATA Request CMD
        if (_my_branch_id < NUM_MAX_SLAVE) {
          // Supply power only if my Branch ID is less than NUM_MAX_SLAVE
          PORTC = PORTC | B00001000; // A3 PC3 to High Start supplying power
                                     //  digitalWrite(A3,HIGH);
        }
        _is_dcdt_ok = true;
        DebugStringPrintln0("DCDT_OK");
      }
      _is_read_polling = true;
      if (_send_flag) {
        // DebugStringPrintln0("Res dat");
        ret        = BCMDRES_READ_ENCODED_DATA;
        _send_flag = false;
      } else {
        // DebugStringPrintln0("Empty Ack");
        ret = BCMDRES_ACK_EMPTY;
      }
      break;
    case BCMDPARAM_READ_IDENTIFICATION:
      ret = BCMDRES_READ_IDENTIFICATION;
      break;
    case BCMDPARAM_READ_DCINFO:
      ret = BCMDRES_READ_DCINFO;
      break;
    case BCMDPARAM_READ_DCMAX:
      ret = BCMDRES_READ_DCMAX;
      break;
    case BCMDPARAM_READ_DCMIN:
      ret = BCMDRES_READ_DCMIN;
      break;
    case BCMDPARAM_READ_DCINI:
      ret = BCMDRES_READ_DCINI;
      break;
    default:
      break;
    }
    break;
  case BCMD_FIND_LED:
    _find_branch_mode = static_cast<BranchCommandParamFind_t>(_cmd_params[0]);
    break;

  // Store received data into RX buffer and later send response against the below commands
  case BCMD_ENCODED_WRITE:
    if (_cmd_params_len > NUM_MAX_TEMP_BUFF) {
      DebugStringPrint0("F: Out of array size!=");
      DebugPlainPrintln0(_cmd_params_len);
    } else if (_translator->hasBinary()) {
      DebugStringPrintln0("F: Cannot parse BCMD_ENCODED_WRITE");
    } else {
      pushToRxRingBuff(_cmd_params, _cmd_params_len, DATA_TYPE_DCDT);
    }
    break;
  case BCMD_BINARY_WRITE:
    if (_cmd_params_len > NUM_MAX_TEMP_BUFF) {
      DebugStringPrint0("F: Out of array size!=");
      DebugPlainPrintln0(_cmd_params_len);
    } else if (!_translator->hasBinary()) {
      DebugStringPrintln0("F: Cannot parse BCMD_BINARY_WRITE");
    } else {
      pushToRxRingBuff(_cmd_params, _cmd_params_len, DATA_TYPE_RAW);
    }
    break;
  default:
    // Protocol 3.1: Other packets from Core should be stored in _rx_buffer
    // which can be read from user module as-is anyway
    if (_cmd_params_len > NUM_MAX_TEMP_BUFF) {
      DebugStringPrintln0("F: Access out of array size!");
      DebugStringPrintln0("_cmd_params_len=");
      DebugPlainPrintln0(_cmd_params_len);
    } else {
      pushToRxRingBuff(_cmd_params, _cmd_params_len, DATA_TYPE_RAW);
    }
    break;
  }

  DebugGPIOLow(PORTD, 2);  // debug D2 PD2
  DebugGPIOHigh(PORTD, 2); // debug D2 PD2

  return ret;
}

bool VivicoreSerial::sendResponse(const BranchCommand_t bcmd, const BranchCommandRes_t res_type) {
  const ChipType_t chip_type        = (0x1E == _signature[0] && 0x95 == _signature[1] && 0x16 == _signature[2])
                                        ? CHIP_TYPE_ATMEGA_328PB
                                        : CHIP_TYPE_ATMEGA_328P;
  const uint8_t    dc_flag_size     = sizeof(uint16_t);
  uint8_t          data_payload_len = 0;
  uint8_t          pkt_payload_len  = 0;
  int16_t          encoding_array[NUM_MAX_DC] = {};
  data_pkt         encodedBuff                = {};
  uint8_t          buf_index                  = 0;
  bool             has_nature_in              = false;
  bool             has_override_ini           = false;

  _data_response[buf_index++] = STX; // STX
  _data_response[buf_index++] = 0;   // Dummy DATA LENGTH
#if defined(BCMD_ON_RES_SUPPORT)
  _data_response[buf_index++] = bcmd;
  _data_response[buf_index++] = _my_branch_id;
#endif

  switch (res_type) {
  case BCMDRES_DISCOVERY_ACK:
    _data_response[buf_index++] = highByte(UART_PROTOCOL_VERSION);
    _data_response[buf_index++] = lowByte(UART_PROTOCOL_VERSION);
    _data_response[buf_index++] = static_cast<uint8_t>(_my_branch_type >> 24);
    _data_response[buf_index++] = static_cast<uint8_t>(_my_branch_type >> 16);
    _data_response[buf_index++] = static_cast<uint8_t>(_my_branch_type >> 8);
    _data_response[buf_index++] = static_cast<uint8_t>(_my_branch_type);
    _data_response[buf_index++] = highByte(LIBRARY_VER_BUILD_NO);
    _data_response[buf_index++] = lowByte(LIBRARY_VER_BUILD_NO);
    _data_response[buf_index++] = highByte(_user_fw_ver);
    _data_response[buf_index++] = lowByte(_user_fw_ver);
    _data_response[buf_index++] = highByte(_min_lib_buildno);
    _data_response[buf_index++] = lowByte(_min_lib_buildno);
    break;
  case BCMDRES_READ_IDENTIFICATION:
    _data_response[buf_index++] = chip_type;
    memcpy(&_data_response[buf_index], _serial_number, sizeof(_serial_number));
    buf_index += sizeof(_serial_number);
    break;
  case BCMDRES_READ_ENCODED_DATA:
    if ((size_t)(_data_len_by_user + 5) > sizeof(_data_response)) {
      // This is unrealistic case
      DebugStringPrintln0("F: Access out of array size!");
      DebugStringPrintln0("_data_len_by_user+5=");
      DebugPlainPrintln0(_data_len_by_user + 5);
      break;
    }
    memcpy(&_data_response[buf_index], _data_by_user, _data_len_by_user);
    buf_index += _data_len_by_user;
    break;
  case BCMDRES_READ_DCINFO:
    _data_response[buf_index++] = _dc_num;
    for (uint8_t i = 0; i < _dc_num; i++) {
      _data_response[buf_index++] = ((static_cast<uint8_t>(_dc_info[i].group_no) & 0xF) << 4) +
                                    ((_dc_info[i].data_nature == DcNature_t::DC_NATURE_IN ? 1 : 0) << 3) +
                                    (static_cast<uint8_t>(_dc_info[i].data_type) & 0x7);
    }
    break;
  case BCMDRES_READ_DCMAX:
    for (int i = 0; i < _dc_num; i++) {
      encoding_array[i] = _dc_info[i].data_max;
    }
    _translator->encode(encoding_array, &encodedBuff);

    pkt_payload_len  = encodedBuff.datalen;
    data_payload_len = (pkt_payload_len > dc_flag_size) ? pkt_payload_len - dc_flag_size : 0;
    memcpy(&_data_response[buf_index], &encodedBuff.data[dc_flag_size], data_payload_len);
    buf_index += data_payload_len;

    // DebugStringPrint0("BCMDRES_READ_DCMAX pkt_payload_len:");
    // DebugPlainPrintln0(pkt_payload_len);
    break;
  case BCMDRES_READ_DCMIN:
    for (int i = 0; i < _dc_num; i++) {
      encoding_array[i] = _dc_info[i].data_min;
    }
    _translator->encode(encoding_array, &encodedBuff);

    pkt_payload_len  = encodedBuff.datalen;
    data_payload_len = (pkt_payload_len > dc_flag_size) ? pkt_payload_len - dc_flag_size : 0;
    memcpy(&_data_response[buf_index], &encodedBuff.data[dc_flag_size], data_payload_len);
    buf_index += data_payload_len;

    // DebugStringPrint0("BCMDRES_READ_DCMIN pkt_payload_len:");
    // DebugPlainPrintln0(pkt_payload_len);
    break;
  case BCMDRES_READ_DCINI:
    for (int i = 0; i < _dc_num; i++) {
      encoding_array[i] = _dc_info[i].data_ini;
      if (_dc_info[i].data_nature == DcNature_t::DC_NATURE_IN) {
        has_nature_in = true;
      }
      if (_scaler_data_for_ini.is_set[i]) {
        has_override_ini  = true;
        encoding_array[i] = _scaler_data_for_ini.body[i];
      }
    }

    if (has_nature_in || has_override_ini) {
      _translator->encode(encoding_array, &encodedBuff);
      pkt_payload_len  = encodedBuff.datalen;
      data_payload_len = (pkt_payload_len > dc_flag_size) ? pkt_payload_len - dc_flag_size : 0;
      memcpy(&_data_response[buf_index], &encodedBuff.data[dc_flag_size], data_payload_len);
      buf_index += data_payload_len;
    }
    break;
  case BCMDRES_ACK_EMPTY:
    break;
  case BCMDRES_NOTHING:
    // no response
    return true;
  default:
    DebugStringPrint0("F: Unknown res type ");
    DebugPlainPrintln0(res_type);
    return false;
  }

  const uint8_t crc_target_length = buf_index;
  _data_response[1]               = buf_index - 1;
  _data_response[buf_index++]     = getCRC8(_data_response, crc_target_length);
  pushToTxRingBuffAndTransmit(_data_response, buf_index);

  DebugGPIOLow(PORTD, 2);  // debug D2 PD2
  DebugGPIOHigh(PORTD, 2); // debug D2 PD2

  return true;
}

void VivicoreSerial::pushToRxRingBuff(const uint8_t *buffer, const uint8_t length, const dataType_t data_type) {
  if (_is_dcdt_ok) {
    // Debug
    // DebugStringPrint0("pushToRxRingBuff: ");
    // for (uint8_t i = 0; i < length; i++) {
    //  DebugPlainPrint0(buffer[i]);
    //}
    // DebugStringPrintln0("");
    // Debug
    uint8_t rlen = length;
    if (rlen > NUM_MAX_UART_PKT_BODY_DATA) {
      DebugStringPrint0("F: Out of array size!=");
      DebugPlainPrintln0(rlen);
      rlen = NUM_MAX_UART_PKT_BODY_DATA;
    }
    unsigned int j = (unsigned int)(_rx_buffer->head + 1) % RING_PKT_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (j != _rx_buffer->tail) {
      memcpy(_rx_buffer->buffer[_rx_buffer->head].data, buffer, rlen);
      _rx_buffer->buffer[_rx_buffer->head].size = rlen;
      _rx_buffer->buffer[_rx_buffer->head].type = data_type;
      _rx_buffer->head                          = j;
    }
  } else {
    DebugPrintln0("E: pushToRxRingBuff called before STEP_DCDT_OK!");
  }
}

#if (CORE_COMM_UART_PORT == 0)
ISR(USART0_RX_vect)
#elif (CORE_COMM_UART_PORT == 1)
ISR(USART1_RX_vect)
#else
#  error Not supported!
#endif
{
  if (active_object->_is_passthru_mode) {
    return;
  }
  DebugGPIOHigh(PORTD, 2); // debug D2 PD2

  uint8_t         uart_stat = *(active_object->_ucsra);
  unsigned char   c         = *(active_object->_udr);
  BranchCommand_t cmd       = BCMD_INVALID;

  if (bit_is_set(uart_stat, active_object->_dor)) {
    DebugStringPrintln0("UCSRxA=DataOverRun");
  }
  if (bit_is_set(uart_stat, active_object->_upe)) {
    DebugStringPrintln0("UCSRxA=ParityError");
  }
  if (bit_is_set(uart_stat, active_object->_fe)) {
    active_object->setSyncBreakReceived();
    // DebugPrintCOBIln("*");
  }

  cmd = active_object->parseCommand(c);
  if (cmd != BCMD_INVALID) {
    const BranchCommandRes_t res = active_object->processCommand(cmd);
    active_object->sendResponse(cmd, res);
  }

  DebugGPIOLow(PORTD, 2); // debug D2 PD2
}

#if (CORE_COMM_UART_PORT == 0)
ISR(USART0_UDRE_vect)
#elif (CORE_COMM_UART_PORT == 1)
ISR(USART1_UDRE_vect)
#else
#  error Not supported!
#endif
{
  if (active_object->_is_passthru_mode) {
    return;
  }
  DebugGPIOHigh(PORTC, 4); // debug A4 PC4

  if (active_object->_tx_buffer->head == active_object->_tx_buffer->tail) {
    // Buffer empty, so disable interrupts
    cbi(*(active_object->_ucsrb), active_object->_udrie);
    // delayMicroseconds(800);
    loop_until_bit_is_set(*(active_object->_ucsra), active_object->_txc);
    PORTB = PORTB & B11111110;          // digitalWrite(PIN_EN_TX, LOW);
    active_object->clearTransmitting(); // Set transmitting to false
  } else {
    // There is more data in the output buffer. Send the next byte
    const unsigned char c           = active_object->_tx_buffer->buffer[active_object->_tx_buffer->tail];
    active_object->_tx_buffer->tail = (active_object->_tx_buffer->tail + 1) & (SERIAL_BUFFER_SIZE - 1);

    // clear the TXC bit -- "can be cleared by writing a one to its bit location"
    sbi(*(active_object->_ucsra), active_object->_txc);
    *(active_object->_udr) = c;
  }

  DebugGPIOLow(PORTC, 4); // debug A4 PC4
}

// Constructors ////////////////////////////////////////////////////////////////

VivicoreSerial::VivicoreSerial(volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
                               volatile uint8_t *ucsrb, volatile uint8_t *ucsrc, volatile uint8_t *udr, uint8_t rxen,
                               uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x, uint8_t txc, uint8_t dor,
                               uint8_t upe, uint8_t fe)
    : _ubrrh(ubrrh), _ubrrl(ubrrl), _ucsra(ucsra), _ucsrb(ucsrb), _ucsrc(ucsrc), _udr(udr), _rxen(rxen), _txen(txen),
      _rxcie(rxcie), _udrie(udrie), _u2x(u2x), _txc(txc), _dor(dor), _upe(upe), _fe(fe),
      _rx_buffer(new RingBuffer<pkt_payload_t, RING_PKT_BUFFER_SIZE>(0, 0)),
      _tx_buffer(new RingBuffer<unsigned char, SERIAL_BUFFER_SIZE>(0, 0)),
      _signature{
        boot_signature_byte_get(0x00), // Value of SIGROW_DEVICEID0
        boot_signature_byte_get(0x02), // Value of SIGROW_DEVICEID1
        boot_signature_byte_get(0x04)  // Value of SIGROW_DEVICEID2
      },
      _serial_number{
        boot_signature_byte_get(0x0E), // Value of SIGROW_SERNUM0
        boot_signature_byte_get(0x0F), // Value of SIGROW_SERNUM1
        boot_signature_byte_get(0x10), // Value of SIGROW_SERNUM2
        boot_signature_byte_get(0x11), // Value of SIGROW_SERNUM3
        boot_signature_byte_get(0x12), // Value of SIGROW_SERNUM4
        boot_signature_byte_get(0x13), // Value of SIGROW_SERNUM5
        boot_signature_byte_get(0x14), // Value of SIGROW_SERNUM6
        boot_signature_byte_get(0x15), // Value of SIGROW_SERNUM7
        boot_signature_byte_get(0x16), // Value of SIGROW_SERNUM8
        boot_signature_byte_get(0x17)  // Value of SIGROW_SERNUM9
      },
      _translator(new DataCodeTranslator()) {

  pinMode(PIN_VIVIWARE_EN_PWR, OUTPUT);
  _is_passthru_mode = GPIOR0 & 0x1;
  if (_is_passthru_mode) {
    // Supply power
    // PORTC = PORTC | B00001000; // A3 PC3 to High Start supplying power
    digitalWrite(PIN_VIVIWARE_EN_PWR, HIGH);
  } else {
    digitalWrite(PIN_VIVIWARE_EN_PWR, LOW);
  }
}

VivicoreSerial::~VivicoreSerial(void) {
  delete _rx_buffer;
  delete _tx_buffer;
  delete _translator;
}

// Private Methods /////////////////////////////////////////////////////////////

void VivicoreSerial::init() {
  pinMode(PIN_VIVIWARE_EN_RX, OUTPUT);
  pinMode(PIN_VIVIWARE_EN_TX, OUTPUT);
  pinMode(PIN_VIVIWARE_DEBUG_LED, OUTPUT);
  digitalWrite(PIN_VIVIWARE_EN_RX, HIGH); // ALWAYS HIGH
  digitalWrite(PIN_VIVIWARE_EN_TX, LOW);  // ONLY HIGH WHEN TRANSMITTING , TO REMOVE

  for (int i = 0; i < NUM_BRTYPE_BYTES; i++) {
    uint8_t brTypeDat = EEPROM.read(E2END - (NUM_BRTYPE_BYTES - i) + 1);
    _saved_branch_type |= (uint32_t)brTypeDat << 8 * (NUM_BRTYPE_BYTES - i - 1);
  }
}

// CRC8 table array
const uint8_t CRC8Table[256]
#ifdef __AVR_ARCH__
  PROGMEM
#endif
  = {
    0x00, 0x85, 0x8F, 0x0A, 0x9B, 0x1E, 0x14, 0x91, 0xB3, 0x36, 0x3C, 0xB9, 0x28, 0xAD, 0xA7, 0x22,
    0xE3, 0x66, 0x6C, 0xE9, 0x78, 0xFD, 0xF7, 0x72, 0x50, 0xD5, 0xDF, 0x5A, 0xCB, 0x4E, 0x44, 0xC1,
    0x43, 0xC6, 0xCC, 0x49, 0xD8, 0x5D, 0x57, 0xD2, 0xF0, 0x75, 0x7F, 0xFA, 0x6B, 0xEE, 0xE4, 0x61,
    0xA0, 0x25, 0x2F, 0xAA, 0x3B, 0xBE, 0xB4, 0x31, 0x13, 0x96, 0x9C, 0x19, 0x88, 0x0D, 0x07, 0x82,

    0x86, 0x03, 0x09, 0x8C, 0x1D, 0x98, 0x92, 0x17, 0x35, 0xB0, 0xBA, 0x3F, 0xAE, 0x2B, 0x21, 0xA4,
    0x65, 0xE0, 0xEA, 0x6F, 0xFE, 0x7B, 0x71, 0xF4, 0xD6, 0x53, 0x59, 0xDC, 0x4D, 0xC8, 0xC2, 0x47,
    0xC5, 0x40, 0x4A, 0xCF, 0x5E, 0xDB, 0xD1, 0x54, 0x76, 0xF3, 0xF9, 0x7C, 0xED, 0x68, 0x62, 0xE7,
    0x26, 0xA3, 0xA9, 0x2C, 0xBD, 0x38, 0x32, 0xB7, 0x95, 0x10, 0x1A, 0x9F, 0x0E, 0x8B, 0x81, 0x04,

    0x89, 0x0C, 0x06, 0x83, 0x12, 0x97, 0x9D, 0x18, 0x3A, 0xBF, 0xB5, 0x30, 0xA1, 0x24, 0x2E, 0xAB,
    0x6A, 0xEF, 0xE5, 0x60, 0xF1, 0x74, 0x7E, 0xFB, 0xD9, 0x5C, 0x56, 0xD3, 0x42, 0xC7, 0xCD, 0x48,
    0xCA, 0x4F, 0x45, 0xC0, 0x51, 0xD4, 0xDE, 0x5B, 0x79, 0xFC, 0xF6, 0x73, 0xE2, 0x67, 0x6D, 0xE8,
    0x29, 0xAC, 0xA6, 0x23, 0xB2, 0x37, 0x3D, 0xB8, 0x9A, 0x1F, 0x15, 0x90, 0x01, 0x84, 0x8E, 0x0B,

    0x0F, 0x8A, 0x80, 0x05, 0x94, 0x11, 0x1B, 0x9E, 0xBC, 0x39, 0x33, 0xB6, 0x27, 0xA2, 0xA8, 0x2D,
    0xEC, 0x69, 0x63, 0xE6, 0x77, 0xF2, 0xF8, 0x7D, 0x5F, 0xDA, 0xD0, 0x55, 0xC4, 0x41, 0x4B, 0xCE,
    0x4C, 0xC9, 0xC3, 0x46, 0xD7, 0x52, 0x58, 0xDD, 0xFF, 0x7A, 0x70, 0xF5, 0x64, 0xE1, 0xEB, 0x6E,
    0xAF, 0x2A, 0x20, 0xA5, 0x34, 0xB1, 0xBB, 0x3E, 0x1C, 0x99, 0x93, 0x16, 0x87, 0x02, 0x08, 0x8D,
};

// Get CRC8 from Table
uint8_t VivicoreSerial::getCRC8(const uint8_t *buff, const size_t size) {
  DebugGPIOHigh(PORTB, 1); // debug D9 PB1
  uint8_t crc8 = 0x00;

  for (size_t i = 0; i < size; i++) {
#ifdef __AVR_ARCH__
    crc8 = pgm_read_byte(&CRC8Table[crc8 ^ buff[i]]);
#else
    crc8 = CRC8Table[crc8 ^ buff[i]];
#endif
  }
  DebugGPIOLow(PORTB, 1); // debug D9 PB1
  return crc8;
}
// Public Methods //////////////////////////////////////////////////////////////

bool VivicoreSerial::begin(const uint32_t branch_type, const uint16_t user_version, const dcInfo_t *dc_info,
                           const uint8_t dc_num, const uint16_t min_lib_buildno) {
  bool ret = true;

  _my_branch_type  = branch_type;
  _user_fw_ver     = user_version;
  _min_lib_buildno = min_lib_buildno;

  init();

  DebugBegin();
#if (CORE_COMM_UART_PORT == 0)
  DebugStringPrintln0("HW Serial1 started");
#elif (CORE_COMM_UART_PORT == 1)
  DebugStringPrintln0("HW Serial0 started");
#else
#  error Not supported!
#endif

  DebugStringPrint0("GPIOR0=");
  DebugHexPrint0(_is_passthru_mode);
  DebugStringPrintln0("");
  if (_is_passthru_mode) {
    DebugStringPrint0("Pass Thru Mode");
    return true;
  }

  DebugStringPrint0("Saved Branch type: 0x");
  DebugHexPrint0(static_cast<uint8_t>(_saved_branch_type >> 24));
  DebugHexPrint0(static_cast<uint8_t>(_saved_branch_type >> 16));
  DebugHexPrint0(static_cast<uint8_t>(_saved_branch_type >> 8));
  DebugHexPrint0(static_cast<uint8_t>(_saved_branch_type));
  DebugStringPrintln0("");

  DebugStringPrint0("Branch type: 0x");
  DebugHexPrint0(static_cast<uint8_t>(_my_branch_type >> 24));
  DebugHexPrint0(static_cast<uint8_t>(_my_branch_type >> 16));
  DebugHexPrint0(static_cast<uint8_t>(_my_branch_type >> 8));
  DebugHexPrint0(static_cast<uint8_t>(_my_branch_type));
  DebugStringPrintln0("");

#ifndef SKIP_VERIFY_BRANCH_TYPE
  if (_saved_branch_type != _my_branch_type) {
    _fatal_mode = true;
    DebugStringPrintln0("Branch type does not match!!");
  }
#endif

  DebugStringPrint0("Protocol ver: ");
  DebugHexPrint0(highByte(UART_PROTOCOL_VERSION));
  DebugStringPrint0(".");
  DebugHexPrint0(lowByte(UART_PROTOCOL_VERSION));
  DebugStringPrintln0("");

  DebugStringPrint0("User FW ver: ");
  DebugHexPrint0(highByte(_user_fw_ver));
  DebugStringPrint0(".");
  DebugHexPrint0(lowByte(_user_fw_ver));
  DebugStringPrintln0("");

  DebugStringPrint0("Signature: ");
  DebugHexPrint0(_signature[0]);
  DebugHexPrint0(_signature[1]);
  DebugHexPrint0(_signature[2]);
  DebugStringPrintln0("");

  DebugStringPrint0("Serial Number: ");
  for (int i = 0; i < NUM_SERIAL_BYTES; i++) {
    DebugHexPrint0(_serial_number[i]);
  }
  DebugStringPrintln0("");

  if (!_translator->init(dc_info, dc_num)) {
    DebugStringPrintln0("F: DC info is not initialized properly");
    ret = false;
  }
  _dc_num  = _translator->getDcNum();
  _dc_info = _translator->getDcInfo();

  for (uint8_t i = 0; i < _dc_num; i++) {
    if (_scaler_data_for_ini.is_set[i]) {
      const int16_t override_ini = _scaler_data_for_ini.body[i];
      if ((override_ini < _dc_info[i].data_min) || (override_ini > _dc_info[i].data_max)) {
        DebugStringPrint0("F: Overriding DC ini is out of range for DC ");
        DebugPlainPrintln0(i + 1);
        ret = false;
      }
    }
  }

  DebugGPIODirectOut(DDRB, 1); // debug D9 PB1
  DebugGPIODirectOut(DDRB, 2); // debug D10 PB2
  DebugGPIODirectOut(DDRB, 5); // debug D13 PB5
  DebugGPIODirectOut(DDRC, 4); // debug A4 PC4
  DebugGPIODirectOut(DDRD, 2); // debug D2 PD2
  DebugGPIODirectOut(DDRD, 3); // debug D3 PD3
  DebugGPIODirectOut(DDRC, 5); // debug A5 PC5

  DebugGPIOLow(PORTB, 1); // debug D9 PB1
  DebugGPIOLow(PORTB, 2); // debug D10 PB2
  DebugGPIOLow(PORTB, 5); // debug D13 PB5
  DebugGPIOLow(PORTC, 4); // debug A4 PC4
  DebugGPIOLow(PORTD, 2); // debug D2 PD2
  DebugGPIOLow(PORTD, 3); // debug D3 PD3
  DebugGPIOLow(PORTC, 5); // debug A5 PC5

  clearTransmitting(); // Set transmitting to false

  setBaud(UART_PROTOCOL_BAUDRATE);

  sbi(*_ucsrb, _rxen);
  sbi(*_ucsrb, _txen);
  sbi(*_ucsrb, _rxcie);
  cbi(*_ucsrb, _udrie);

  return ret;
}

void VivicoreSerial::setBaud(unsigned long baud) {
  uint16_t baud_setting;
#if F_CPU == 16000000UL
  bool use_u2x = true;

  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    use_u2x = false;
  }
#endif

  *_ucsra      = 1 << _u2x;
  baud_setting = (F_CPU / 4 / baud / 2) - 1;

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;
}

void VivicoreSerial::end(void) {
  if (_is_passthru_mode) {
    return;
  }
  // wait for transmission of outgoing data
  while (_tx_buffer->head != _tx_buffer->tail)
    ;

  cbi(*_ucsrb, _rxen);
  cbi(*_ucsrb, _txen);
  cbi(*_ucsrb, _rxcie);
  cbi(*_ucsrb, _udrie);

  // clear any received data
  _rx_buffer->head = _rx_buffer->tail;
}

bool VivicoreSerial::pullFromRxRingBuff(data_pkt *raw_data) {
  bool ret = true;

  raw_data->datalen = 0;

  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer->head == _rx_buffer->tail) {
    return true;
  }

  const pkt_payload_t *pkt = &_rx_buffer->buffer[_rx_buffer->tail];
  _rx_buffer->tail         = (unsigned int)(_rx_buffer->tail + 1) % RING_PKT_BUFFER_SIZE;

  if (pkt->size <= NUM_MAX_UART_PKT_BODY_DATA) {
    raw_data->datalen = pkt->size;
  } else {
    DebugStringPrint0("F: Out of array size ");
    DebugPlainPrint0(pkt->size);
    DebugStringPrintln0(" and some data is dropped");
    raw_data->datalen = NUM_MAX_UART_PKT_BODY_DATA;
    ret               = false;
  }

  memcpy(raw_data->data, pkt->data, raw_data->datalen);

  return ret;
}

AvailableNum_t VivicoreSerial::available(void) {
  AvailableNum_t ret = {};

  if (_is_passthru_mode) {
    return ret;
  }

  pullFromRxRingBuff(&_raw_data_by_core);
  if (_raw_data_by_core.datalen == 0) {
    _scaler_data_by_core.dc_nums_count = 0;
    return ret;
  }

  // Only a raw data in a packet
  ret.raw = 1;

  // Decode raw data and get DC data as far as possible
  _translator->decode(&_raw_data_by_core, _scaler_data_by_core.body, _scaler_data_by_core.dc_nums,
                      &_scaler_data_by_core.dc_nums_count);
  ret.scaler = static_cast<size_t>(_scaler_data_by_core.dc_nums_count);

  return ret;
}

ScalerData_t VivicoreSerial::read(void) {
  ScalerData_t ret = {};

  if (_is_passthru_mode) {
    ret.success = true;
    return ret;
  }

  if (_scaler_data_by_core.dc_nums_count == 0) {
    return ret;
  }

  // DataCodeTranslator::decode() already checked DC number range
  ret.dc_n = _scaler_data_by_core.dc_nums[--_scaler_data_by_core.dc_nums_count];
  ret.data = static_cast<int32_t>(_scaler_data_by_core.body[ret.dc_n - 1]);

  if ((ret.data < _dc_info[ret.dc_n - 1].data_min) || (ret.data > _dc_info[ret.dc_n - 1].data_max)) {
    DebugStringPrint0("F: Read data is out of range for DC ");
    DebugPlainPrintln0(ret.dc_n);
  } else if (_dc_info[ret.dc_n - 1].data_nature != DcNature_t::DC_NATURE_IN) {
    DebugStringPrint0("F: Reading data from DC_NATURE_OUT ");
    DebugPlainPrintln0(ret.dc_n);
  } else if (_dc_info[ret.dc_n - 1].data_type == DcType_t::DC_TYPE_BINARY) {
    DebugStringPrint0("F: Invalid data type of DC ");
    DebugPlainPrintln0(ret.dc_n);
  } else {
    ret.success = true;
  }

  return ret;
}

RawData_t VivicoreSerial::readRaw(void) {
  RawData_t ret = {};

  if (_is_passthru_mode) {
    ret.success = true;
    return ret;
  }

  if (_raw_data_by_core.datalen == 0) {
    return ret;
  }

  ret.data_len =
    (_raw_data_by_core.datalen > sizeof(ret.data) ? sizeof(ret.data) : static_cast<int>(_raw_data_by_core.datalen));
  memcpy(ret.data, _raw_data_by_core.data, ret.data_len);
  memset(&_raw_data_by_core, 0, sizeof(_raw_data_by_core));
  ret.success = true;

  return ret;
}

bool VivicoreSerial::flush(void) {
  data_pkt writing_data = {};
  bool     ret          = true;

  if (_is_passthru_mode) {
    return true;
  }
  if (_dc_info == nullptr) {
    DebugStringPrintln0("F: No DC info");
    return false;
  }

  ret = _translator->encode(_scaler_data_by_user.body, _scaler_data_by_user.is_set, &writing_data);
  if ((writing_data.datalen == 0) && (_raw_data_by_user.datalen > 0)) {
    writing_data = _raw_data_by_user;
    ret          = true;
  }

  if (writing_data.datalen > 0) {
    _data_len_by_user = writing_data.datalen;
    if ((size_t)_data_len_by_user > sizeof(_data_by_user)) {
      DebugStringPrint0("F: Out of array size! _data_len_by_user=");
      DebugPlainPrintln0(_data_len_by_user);
      _data_len_by_user = sizeof(_data_by_user);
      ret               = false;
    }

    memcpy(_data_by_user, writing_data.data, _data_len_by_user);
    _send_flag = true;
  }

  // If we have never written a byte, no need to flush.
  if (_send_flag || isTransmitting()) {
    // Wait for DCDT OK
    while (!_is_dcdt_ok) {
      if (sleep_in_flush) {
        sleep_in_flush();
      }
    }
    while (_send_flag) {
      if (sleep_in_flush) {
        sleep_in_flush();
      }
    }
    // UDR is kept full while the buffer is not empty, so TXC triggers when EMPTY
    // && SENT
    while (isTransmitting() && !bitRead(*_ucsra, _txc)) {
      if (sleep_in_flush) {
        sleep_in_flush();
      }
    }
    clearTransmitting(); // Set transmitting to false
  }

  // Reset buffer for both of scaler and raw at flushing
  memset(_scaler_data_by_user.is_set, 0, sizeof(_scaler_data_by_user.is_set));
  _raw_data_by_user.datalen = 0;

  // DebugStringPrintln0("ret flush");
  return ret;
}

bool VivicoreSerial::writeRaw(const uint8_t *data, const size_t data_len, const dataType_t data_type) {
  bool ret = true;

  if (_is_passthru_mode) {
    return true;
  }

  // TODO: Not implemented yet
  if (data_type != DATA_TYPE_DCDT) {
    return false;
  }

  if (data_len <= sizeof(_raw_data_by_user.data)) {
    _raw_data_by_user.datalen = data_len;
  } else {
    _raw_data_by_user.datalen = sizeof(_raw_data_by_user.data);
    ret                       = false;
    DebugStringPrint0("F: Writing data ");
    DebugPlainPrint0(data_len);
    DebugPlainPrint0(" is over limit ");
    DebugPlainPrintln0(sizeof(_raw_data_by_user.data));
  }

  memcpy(&_raw_data_by_user.data, data, _raw_data_by_user.datalen);
  return ret;
}

bool VivicoreSerial::write(const uint8_t dc_n, const int32_t data_scaler) {
  scalerDataW_t *const scaler_data = (_dc_info == nullptr) ? &_scaler_data_for_ini : &_scaler_data_by_user;

  if (_is_passthru_mode) {
    return true;
  }

  if ((dc_n <= 0) || (dc_n > NUM_MAX_DC)) {
    return false;
  }

  scaler_data->body[dc_n - 1]   = static_cast<int16_t>(data_scaler);
  scaler_data->is_set[dc_n - 1] = true;
  return true;
}

bool VivicoreSerial::isInFatalError(void) {
  return _fatal_mode;
}

void VivicoreSerial::pushToTxRingBuffAndTransmit(const uint8_t *buffer, const uint8_t datalen) {
  PORTB = PORTB | B00000001; // digitalWrite(PIN_EN_TX, HIGH);

  setSyncBreak();

  sbi(*_ucsrb, _udrie);
  setTransmitting(); // Set transmitting to true
  // clear the TXC bit -- "can be cleared by writing a one to its bit location"
  sbi(*_ucsra, _txc);

  DebugPrintCIBO("CIBO:");
  for (int i = 0; i < datalen; i++) {
    DebugHexPrintCIBO(buffer[i]);
    pushToTxRingBuff(buffer[i]);
  }
  DebugPrintCIBOln("");
  // delayMicroseconds(datalen*690);
  //  while (!(*_ucsra & (1 << UDRE0)))  // Wait for empty transmit buffer
  //     *_ucsra |= 1 << _txc;  // mark transmission not complete
  // while (!(*_ucsra & (1 << _txc)));   // Wait for the transmission to complete
  //
  // delay(10);
  // flush();
  // loop_until_bit_is_set(*_ucsra, UDRE0);
  DebugGPIOLow(PORTD, 2);  // debug D2 PD2
  DebugGPIOHigh(PORTD, 2); // debug D2 PD2
}

void VivicoreSerial::setSyncBreak(void) {
  DebugGPIOHigh(PORTC, 5); // debug A5 PC5
  // wait for transmission of outgoing data

  // UDR is kept full while the buffer is not empty, so TXC triggers when EMPTY
  // && SENT
  while (isTransmitting() && !bitRead(*_ucsra, _txc))
    ;
  clearTransmitting(); // Set transmitting to false

  DebugGPIOWrite(PORTD, 3, 1); // debug D3 PD3
  DebugGPIOWrite(PORTD, 3, 0); // debug D3 PD3

  cbi(*_ucsrb, _rxen);
  cbi(*_ucsrb, _txen);
  cbi(*_ucsrb, _rxcie);

  setBaud(SLOW_BAUD); // Set slow baud rate to generate sync_break

  sbi(*_ucsrb, _txen);

  sbi(*_ucsra, _txc);

  DebugGPIOWrite(PORTD, 3, bitRead(*_ucsra, _txc)); // debug D3 PD3

  // sync_break;
  *_udr = 0;

  DebugGPIOWrite(PORTD, 3, bitRead(*_ucsra, _txc)); // debug D3 PD3

  // loop_until_bit_is_set(*_ucsra, UDRE0);
  // loop_until_bit_is_set(*_ucsra, _txc);
  while (!bitRead(*_ucsra, _txc)) {
    DebugGPIOWrite(PORTD, 3, bitRead(*_ucsra, _txc)); // debug D3 PD3
    DebugGPIOHigh(PORTB, 1);                          // debug D9 PB1
    DebugGPIOLow(PORTB, 1);                           // debug D9 PB1
  }
  DebugGPIOWrite(PORTD, 3, bitRead(*_ucsra, _txc)); // debug D3 PD3

  cbi(*_ucsrb, _txen);

  setBaud(UART_PROTOCOL_BAUDRATE); // Revert baud rate

  sbi(*_ucsrb, _rxen);
  sbi(*_ucsrb, _txen);
  sbi(*_ucsrb, _rxcie);
  DebugGPIOLow(PORTC, 5); // debug A5 PC5
}

size_t VivicoreSerial::pushToTxRingBuff(const uint8_t c) {
  unsigned int i = (_tx_buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // If the output buffer is full, there's nothing for it other than to return
  if (i == _tx_buffer->tail) {
    return 0;
  }

  _tx_buffer->buffer[_tx_buffer->head] = c;
  _tx_buffer->head                     = i;

  return 1;
}

inline void VivicoreSerial::controlLedBlink(const bool *timing_table, const uint8_t max_count) {
  static uint8_t       cnt  = 0;
  static unsigned long prev = 0; // Workaround for undefined reference to '__cxa_guard_acquire' '__cxa_guard_release'
  static bool *        prevTimingTablePtr = NULL;

  if (timing_table == NULL) {
    return;
  }

  if (prevTimingTablePtr != timing_table) {
    // Reset cnt if different timing_table was provided
    cnt                = 0;
    prevTimingTablePtr = const_cast<bool *>(timing_table);
  }

  const unsigned long now = millis();
  // Dont care about millis overflows https://garretlab.web.fc2.com/arduino/lab/millis/
  if (now - prev > LED_BLINK_INTERVAL) {
    prev = now;
    if (cnt == max_count) {
      cnt = 0;
    }
    const bool led_state = timing_table[cnt];
    setIndicatorLed(led_state);
    cnt++;
  }
}

inline void VivicoreSerial::controlLedOnOff(const bool stop_blink) {
  static bool startup_led_ctrl_done = false;
  if (stop_blink) {
    // MUST control LED just after LED blink stop
    if (startup_led_ctrl_done || millis() > STARTUP_LED_ON_DURATION) {
      // Turn off LED STARTUP_LED_ON_DURATION msec after boot-up
      startup_led_ctrl_done = true;
      setIndicatorLed(false);
    } else {
      setIndicatorLed(true);
    }
  } else if (!startup_led_ctrl_done && millis() > STARTUP_LED_ON_DURATION) {
    // In normal operation mode, cotrol LED only STARTUP_LED_ON_DURATION msec after boot-up
    // Turn off LED STARTUP_LED_ON_DURATION msec after boot-up
    startup_led_ctrl_done = true;
    setIndicatorLed(false);
  }
}

void VivicoreSerial::manageLedState(void) {
  static const bool timingTableFindBranchNormal[] = {1, 1, 0, 0};
  static const bool timingTableFindBranchSlow[]   = {1, 1, 1, 1, 0, 0, 0, 0};
  static const bool timingTableFindBranchRapid[]  = {1, 0};
  static const bool timingTableFatal[]            = {1, 0, 1, 0, 0, 0, 0, 0, 0, 0};

  if (_find_branch_mode != BCMDPARAM_FIND_BLINK_OFF) {
    _dominate_led = true;
    if (_find_branch_mode == BCMDPARAM_FIND_BLINK_SLOW) {
      controlLedBlink(timingTableFindBranchSlow, countof(timingTableFindBranchSlow));
    } else if (_find_branch_mode == BCMDPARAM_FIND_BLINK_RAPID) {
      controlLedBlink(timingTableFindBranchRapid, countof(timingTableFindBranchRapid));
    } else {
      controlLedBlink(timingTableFindBranchNormal, countof(timingTableFindBranchNormal));
    }
  } else {
    if (_fatal_mode) {
      _dominate_led = true;
      controlLedBlink(timingTableFatal, countof(timingTableFatal));
    } else if (_dominate_led) {
      _dominate_led = false;
      controlLedOnOff(true);
    } else {
      controlLedOnOff(false);
    }
  }
}
