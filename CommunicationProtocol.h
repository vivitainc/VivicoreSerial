/*
  CommunicationProtocol.h - Protocol definitions for VIVIWARE Cell Branch
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

  Modified 06 March 2021 by VIVITA Japan, Inc.
*/

#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

/***********************************************************************************************//**
 * BLE communication protocol version
 *
 * NOTE:
 * Higher byte should be incremented if any of the following condition.
 *  - New BLE services/characteristics are added
 *  - Any of the BLE services/characteristics is removed
 *  - Any of the BLE characteristics permission is changed and it influences the behavior
 * Lower byte should be incremented if any of the following condition.
 *  - Internal structure and/or member of packet for any command is added, removed, or changed
 *
 * Synchronize this version number defined on some files with the instruction written on
 * update_version.sh.
 **************************************************************************************************/
#define BLE_PROTOCOL_VERSION (uint16_t)(0x0603)

/***********************************************************************************************//**
 * Communication spec for App/Core
 **************************************************************************************************/
#define NUM_MAX_BLE_PKT      (20)                  // Max BLE packet size                          = ACMD packet HEADER + BODY
#define NUM_MAX_BLE_PKT_BODY (NUM_MAX_BLE_PKT - 2) // Max BLE packet size - ACMD(1) - BRANCH_ID(1) = ACMD packet BODY

typedef enum {
  ACMD_BRANCHTYPE = 0x00,
  ACMD_DISCONNECT = 0x01,
  ACMD_DATA = 0x02,
  ACMD_ALLDISCONNECT = 0x03,
  ACMD_DCINFO = 0x04,
  ACMD_BINARY = 0x05,
  ACMD_FIND_BRANCH = 0x06,
  ACMD_DCMAX = 0x07,
  ACMD_DCMIN = 0x08,
  ACMD_UNIQUE_ID = 0x09,
  ACMD_OTA_STATUS = 0x0A,
  ACMD_DCINI = 0x0B,
  ACMD_FIND_CORE = 0x0C,
  ACMD_SHUTDOWN = 0xF0,
} AppCommand_t;

// Command parameter for ACMD_SHUTDOWN
typedef enum {
  ACMDPARAM_SHUTDOWN_BATTERY_LOW = 0x01,
  ACMDPARAM_SHUTDOWN_SWICTCH_OFF = 0x02,
} AppCommandParamShutdown_t;

// Command parameter for ACMD_FIND_CORE/BRANCH
typedef enum {
  ACMDPARAM_FIND_BLINK_OFF = 0x00,
  ACMDPARAM_FIND_BLINK_NORMAL = 0x01,
  ACMDPARAM_FIND_BLINK_SLOW = 0x02,
  ACMDPARAM_FIND_BLINK_RAPID = 0x03,
} AppCommandParamFind_t;

/***********************************************************************************************//**
 * UART communication protocol version
 *
 * NOTE:
 * Higher byte should be incremented if any of the following condition.
 *  - Any of the UART configuration like baudrate is changed and it influences the behavior
 * Lower byte should be incremented if any of the following condition.
 *  - The internal structure and/or member of packet for any command is added, removed, or changed
 *
 * Synchronize this version number defined on some files with the instruction written on
 * update_version.sh.
 **************************************************************************************************/
#define UART_PROTOCOL_VERSION (uint16_t)(0x0605)

#define UART_PROTOCOL_VERSION_DCINI          (uint16_t)(0x0603)  // DC_INI is supported on this version or later
#define UART_PROTOCOL_VERSION_NO_BCMD_ON_RES (uint16_t)(0x0604)  // No BCMD header on response packet on this version or later

/***********************************************************************************************//**
 * UART communication spec
 **************************************************************************************************/
#define UART_PROTOCOL_BAUDRATE (100000)

/***********************************************************************************************//**
 * Communication spec for Core/Branch
 **************************************************************************************************/
#define NUM_MAX_UART_PKT_BODY      (NUM_MAX_BLE_PKT_BODY + 2)  // Max BLE packet data payload + BCMD(1) + BRANCH_ID(1)            = BCMD packet BODY
#define NUM_MAX_UART_PKT           (NUM_MAX_UART_PKT_BODY + 3) // Max UART packet data payload + STX(1) + DATA_LENGTH(1) + CRC(1) = BCMD packet HEADER + BODY + FOOTER
#define NUM_MAX_UART_PKT_BODY_DATA (NUM_MAX_BLE_PKT_BODY)      // maximum number of I/O Data payload without checksum

typedef enum {
  STX = 0xC1,
} HeaderType_t;

// Central to Peripherals(Core) and Peripherals(Core) to Branch commands
typedef enum {
  BCMD_DISCOVERY = 0x00,
  BCMD_ENCODED_WRITE = ACMD_DATA,
  BCMD_DCINFO = ACMD_DCINFO,
  BCMD_BINARY_WRITE = ACMD_BINARY,
  BCMD_FIND_LED = ACMD_FIND_BRANCH,
  BCMD_READ = 0x32,
  BCMD_BRANCH_DFU_START = 0x64,
  BCMD_RESET = 0x99,
  BCMD_INVALID = 0xFF,
} BranchCommand_t;

// Command parameter for BCMD_READ
typedef enum {
  BCMDPARAM_RESERVED = 0x00,
  BCMDPARAM_READ_IDENTIFICATION = 0x01,
  BCMDPARAM_READ_ENCODED_DATA = 0x02,
  BCMDPARAM_READ_DCINFO = 0x03,
  BCMDPARAM_READ_DCMAX = 0x04,
  BCMDPARAM_READ_DCMIN = 0x05,
  BCMDPARAM_READ_DCINI = 0x06,
} BranchCommandParamRead_t;

// Command parameter for BCMD_FIND_LED
typedef enum {
  BCMDPARAM_FIND_BLINK_OFF = ACMDPARAM_FIND_BLINK_OFF,
  BCMDPARAM_FIND_BLINK_NORMAL = ACMDPARAM_FIND_BLINK_NORMAL,
  BCMDPARAM_FIND_BLINK_SLOW = ACMDPARAM_FIND_BLINK_SLOW,
  BCMDPARAM_FIND_BLINK_RAPID = ACMDPARAM_FIND_BLINK_RAPID,
} BranchCommandParamFind_t;

// Command response type
typedef enum {
  BCMDRES_RESERVED = 0x00,
  BCMDRES_READ_IDENTIFICATION = BCMDPARAM_READ_IDENTIFICATION,
  BCMDRES_READ_ENCODED_DATA = BCMDPARAM_READ_ENCODED_DATA,
  BCMDRES_READ_DCINFO = BCMDPARAM_READ_DCINFO,
  BCMDRES_READ_DCMAX = BCMDPARAM_READ_DCMAX,
  BCMDRES_READ_DCMIN = BCMDPARAM_READ_DCMIN,
  BCMDRES_READ_DCINI = BCMDPARAM_READ_DCINI,
  BCMDRES_DISCOVERY_ACK = 0xAA,
  BCMDRES_ACK_EMPTY = 0xBB,
  BCMDRES_NOTHING = 0xFF,
} BranchCommandRes_t;

// UART communication protocol packet format
typedef struct {
  uint8_t bcmd;
  uint8_t branch_id;
} BcmdPktHdr_t;

typedef struct {
  BcmdPktHdr_t hdr;
  uint8_t body[NUM_MAX_UART_PKT_BODY_DATA];
} BcmdReqPkt_t;

// TODO: deprecated packet structure
typedef struct {
  BcmdPktHdr_t hdr;
  uint8_t body[NUM_MAX_UART_PKT_BODY_DATA];
} BcmdResWithBcmdHdrPkt_t;

typedef struct {
  uint8_t body[NUM_MAX_UART_PKT_BODY_DATA];
} BcmdResPkt_t;

typedef struct {
  uint8_t stx;
  uint8_t len;
} UartPktHdr_t;

typedef struct {
  UartPktHdr_t hdr;
  BcmdReqPkt_t body;
} UartReqPkt_t;

typedef struct {
  UartPktHdr_t hdr;
  BcmdResPkt_t body;
} UartResPkt_t;

/***********************************************************************************************//**
 * Common communication spec applicable for both App/Core and Core/Branch
 **************************************************************************************************/
#define NUM_MAX_DC                      (16)                       // Maximum number of DC info in one Branch
#define NUM_MAX_PKT_BODY_DATA_DC_INI    (NUM_MAX_BLE_PKT_BODY - 2) // Maximum number of DC ini/min/max data size
#define NUM_MAX_PKT_BODY_DATA_DC_MIN    (NUM_MAX_PKT_BODY_DATA_DC_INI)
#define NUM_MAX_PKT_BODY_DATA_DC_MAX    (NUM_MAX_PKT_BODY_DATA_DC_INI)

typedef enum {
  BRANCH_ID_BROADCAST = 0x00,
  BRANCH_ID_ALL = BRANCH_ID_BROADCAST,
  BRANCH_ID_ROOT = 0x01,
  BRANCH_ID_2 = 0x02,
  BRANCH_ID_3 = 0x03,
  BRANCH_ID_4 = 0x04,
  BRANCH_ID_5 = 0x05,
  BRANCH_ID_MAX = BRANCH_ID_5,
  BRANCH_ID_IGNORED = 0xFF,
} BranchId_t;

typedef enum {
  CHIP_TYPE_ATMEGA_328P = 0x00,
  CHIP_TYPE_ATMEGA_328PB = 0x01,
} ChipType_t;

typedef enum {
  DC_GROUP_FOR_SYSTEM = 0,
  DC_GROUP_1,
  DC_GROUP_2,
  DC_GROUP_3,
  DC_GROUP_4,
  DC_GROUP_5,
  DC_GROUP_6,
  DC_GROUP_7,
  DC_GROUP_8,
  DC_GROUP_9,
  DC_GROUP_10,
  DC_GROUP_11,
  DC_GROUP_12,
  DC_GROUP_13,
  DC_GROUP_14,
  DC_GROUP_15,
} DcGroup_t;

typedef enum {
  DC_NATURE_OUT = 0,
  DC_NATURE_IN  = 1,
} DcNature_t;

typedef enum {
  DC_TYPE_BOOLEAN       = 0x01,
  DC_TYPE_ANALOG_1BYTE  = 0x05,
  DC_TYPE_ANALOG_2BYTES = 0x06,
  DC_TYPE_BINARY        = 0x07,
} DcType_t;

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_PROTOCOL_H
