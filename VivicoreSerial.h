/*
  VivicoreSerial.h - Hardware serial library for VIVIWARE Cell Branch
  Copyright (c) 2021 VIVITA Japan, Inc.  All right reserved.

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

  Modified 06 March 2021 by VIVITA Japan, Inc. based on SlashDevin/NeoHWSerial

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

#ifndef VivicoreSerial_h
#define VivicoreSerial_h

#define BOARD_REV_FP1_EVT (1)
#define BOARD_REV_FP1_DVT (2)
#define BOARD_REV_FP1_PVT (3)
#define BOARD_REV         (BOARD_REV_FP1_DVT)

#ifndef CORE_COMM_UART_PORT
#  define CORE_COMM_UART_PORT (1)  // Custom cell should have 1
#endif

#include <Arduino.h>
#include "CommunicationProtocol.h"
#include "VivicoreSerialVersion.h"
#include "VivicoreSerialDebug.h"

// IO pin number definitions
#define PIN_DEBUG_LED (6)
#define PIN_EN_RX     (7)
#define PIN_EN_TX     (8)

#define NUM_DECODED_BUFF (NUM_MAX_UART_PKT_BODY_DATA * 2) // maximum buffer for input of dcdtencoder and output of dcdtdecoder
#define BINARY_SUPPORT (1) // CMD 0x05 Binary support
#define NUM_SERIAL_BYTES (10)
#define NUM_BRTYPE_BYTES (4)

// defines for Viviware communication protocol
#define NUM_MAX_SEND_BUFF (NUM_MAX_UART_PKT) // maximum byte of send packet
#define NUM_MAX_TEMP_BUFF (30) // maximum byte of tempBuffer
#define NUM_MAX_READ_BUFF (64) // To be removed: maximum byte of RingBuffer<unsigned char>

#define NUM_DATA_PKT_STRUCT_LEN (NUM_DECODED_BUFF > NUM_MAX_UART_PKT ? NUM_DECODED_BUFF : NUM_MAX_UART_PKT)

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#if (RAMEND < 1000)
#define SERIAL_BUFFER_SIZE (1UL << 4)
#else
#define SERIAL_BUFFER_SIZE (1UL << 6)
#endif

#define RING_PKT_BUFFER_SIZE (10)

// Template functions
// Get the number of element for given array
template<typename T, size_t N> size_t countof(const T (&)[N]) { return N; }

template <typename T, size_t SIZE>
struct RingBuffer {
  T buffer[SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;

  RingBuffer(const unsigned int HEAD, const unsigned int TAIL) :
    buffer{},
    head(HEAD),
    tail(TAIL) {}
};

struct data_pkt {
  uint8_t data[NUM_DATA_PKT_STRUCT_LEN];
  uint8_t datalen;
};

typedef enum {
  eSizeBool = 0,
  eSizeInt8,
  eSizeInt16,
  eSizeBinary = 0xFF
} dataSize_t;

typedef struct {
  DcGroup_t group_no;
  DcNature_t data_nature;
  DcType_t data_type;
  int16_t data_min;
  int16_t data_max;
  int16_t data_ini;
} dcInfo_t;

typedef struct {
  bool set;
  int16_t data_ini;
} overrideIni_t;

typedef enum {
  eRawData = 0,
  eDcdt
} dataType_t;

struct pkt_payload_t {
  uint8_t data[NUM_MAX_UART_PKT_BODY_DATA];
  uint16_t size;
  dataType_t type;
};

class VivicoreSerial {
public:
  VivicoreSerial(
    volatile uint8_t *ubrrh, // USART baudrate register high
    volatile uint8_t *ubrrl, // USART baudrate register low
    volatile uint8_t *ucsra, // USART Control and Status Registers
    volatile uint8_t *ucsrb, // USART Control and Status Registers
    volatile uint8_t *ucsrc, // USART Control and Status Registers
    volatile uint8_t *udr,   // USART Data Register
    uint8_t rxen,   // Receiver Enable bit on UCSR0B, UCSR1B
    uint8_t txen,   // Transmitter Enable bit on UCSR0B, UCSR1B
    uint8_t rxcie,  // RX Complete Interrupt Enable bit on UCSR0B, UCSR1B
    uint8_t udrie,  // Data Register Empty Interrupt Enable bit on UCSR0A, UCSR1A
    uint8_t u2x,    // Double the USART Transmission Speed bit on UCSR0A, UCSR1A
    uint8_t txc,    // USART Transmit Complete bit on UCSR0A, UCSR1A
    uint8_t dor,    // Data OverRun bit on UCSR0A, UCSR1A
    uint8_t upe,    // USART Parity Error bit on UCSR0A, UCSR1A
    uint8_t fe,     // Frame Error bit on UCSR0A, UCSR1A
    bool infinite_wait = true); // Wait infinite if WDT reset
  virtual ~VivicoreSerial(void);

  void begin(const uint32_t brType, const uint16_t nVer,
             const dcInfo_t *s_dc_info, const uint8_t nNum_dc,
             const uint16_t min_lib_buildno = 0);

  void end(void);
  uint8_t getCRC8(const uint8_t *buff, const size_t size);
  void valuedecoder(const uint8_t *buffer, uint8_t &bcursor, const uint8_t *dcsize, const uint8_t i, data_pkt &recv_pkt, uint8_t &rcursor, uint8_t &bit_cnt);
  data_pkt dcdtencoder(const uint8_t *buffer, const uint8_t datalen, const uint8_t *dcsize, const uint8_t numdc);
  data_pkt dcdtdecoder(const uint8_t *buffer, const uint8_t len, const uint8_t *dcsize, const uint8_t numdc);
  virtual int available(void);
  virtual int read(void);
  virtual void flush(void);
  void write(const uint8_t *buffer, const uint8_t datalen, const dataType_t dataType = eDcdt);
  void writeBinary(const uint8_t *buffer, const uint8_t datalen) {
    write(buffer, datalen, eRawData);
  }
  bool isInFatalError(void);

  // Below members are NOT for user but for interrupt handler to access VivicoreSerial internal state
  BranchCommand_t parseCommand(const uint8_t c);
  BranchCommandRes_t processCommand(const BranchCommand_t cmd);
  void sendResponse(const BranchCommand_t bcmd, const BranchCommandRes_t res_type);

  uint8_t* assignSize(const dcInfo_t *sDcInfo, const uint8_t &num_dc);
  void clearTransmitting(void);
  void setSyncBreakReceived(void);
  void setOverrideIni(const uint8_t dc_idx, const int16_t val, const dcInfo_t *s_dc_info, const uint8_t nNum_dc);

  volatile uint8_t *_ubrrh; // USART baudrate register high
  volatile uint8_t *_ubrrl; // USART baudrate register low
  volatile uint8_t *_ucsra; // USART Control and Status Registers
  volatile uint8_t *_ucsrb; // USART Control and Status Registers
  volatile uint8_t *_ucsrc; // USART Control and Status Registers
  volatile uint8_t *_udr;   // USART Data Register
  uint8_t _rxen;            // Receiver Enable bit on UCSR0B, UCSR1B
  uint8_t _txen;            // Transmitter Enable bit on UCSR0B, UCSR1B
  uint8_t _rxcie;           // RX Complete Interrupt Enable bit on UCSR0B, UCSR1B
  uint8_t _udrie;           // Data Register Empty Interrupt Enable bit on UCSR0A, UCSR1A
  uint8_t _u2x;             // Double the USART Transmission Speed bit on UCSR0A, UCSR1A
  uint8_t _txc;             // USART Transmit Complete bit on UCSR0A, UCSR1A
  uint8_t _dor;             // Data OverRun bit on UCSR0A, UCSR1A
  uint8_t _upe;             // USART Parity Error bit on UCSR0A, UCSR1A
  uint8_t _fe;              // Frame Error bit on UCSR0A, UCSR1A

  RingBuffer<pkt_payload_t, RING_PKT_BUFFER_SIZE> *rx_buffer;
  RingBuffer<unsigned char, SERIAL_BUFFER_SIZE> *tx_buffer;

  bool isPassthruMode;

private:
  uint8_t data_by_user[NUM_MAX_SEND_BUFF];
  uint8_t data_response[NUM_MAX_SEND_BUFF];
  uint8_t *cmd_params;
  uint8_t cmd_params_len;
  uint8_t currIOlen;

  uint8_t _num_dc;
  dcInfo_t dcInfo_[NUM_MAX_DC] = {};
  overrideIni_t overrideIni[NUM_MAX_DC] = {};
  uint8_t _dc_size[NUM_MAX_DC] = {};
  bool dominate_led = false;

  uint16_t read_data_remaining_;
  data_pkt decoded_data_;

  volatile bool is_dcdt_ok;
  volatile uint8_t send_flag;   // Written data is available for transmission if true
  uint8_t flag_readpoll_;
  bool in_transmitting;         // UART transmission is on-going if true
  bool is_sync_break_received;  // sync break is issued if true
  bool infinite_wait_reset;     // Infinitely wait for reset by WD after received RESET CMD if true

  uint8_t my_branch_id;
  uint32_t branch_type_;
  const uint8_t signature_[3];
  const uint8_t serial_number_[NUM_SERIAL_BYTES];
  uint16_t user_fw_ver_;
  uint16_t min_lib_buildno_;
  bool fatal_mode_;
  BranchCommandParamFind_t find_branch_mode_;
  uint32_t savedBranchType_;

  VivicoreSerial(const VivicoreSerial &);
  VivicoreSerial &operator=(const VivicoreSerial &);

  void init(void);
  data_pkt makebuffer(const uint16_t *value_array, const uint8_t *dcsize, const uint8_t numdc);
  data_pkt pullFromRxRingBuff(void);
  void pushToRxRingBuff(const uint8_t *buffer, const uint8_t length, const dataType_t dataType);
  size_t pushToTxRingBuff(const uint8_t c);
  void pushToTxRingBuffAndTransmit(const uint8_t *buffer, const uint8_t datalen);

  void setTransmitting(void);
  bool isTransmitting(void);
  bool isSyncBreakReceived(void);

  void setBaud(unsigned long baud);
  void setSyncBreak(void);

  void controlLedBlink(const bool *timingTable, const uint8_t maxCnt);
  void controlLedOnOff(const bool stop_blink);
  void manageLedState(void);
};

extern VivicoreSerial Vivicore;

#endif
