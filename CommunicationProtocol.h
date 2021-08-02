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
*/

/**
 * @file CommunicationProtocol.h
 * @brief Communication protocol definitions for VIVIWARE Cell Branch and Custom
 */

#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

/**
 * @brief BLE communication protocol version
 *
 * Higher byte of spec version should be incremented if any of the following changes is applied.
 * - New BLE services/characteristics are added
 * - Any of the BLE services/characteristics is removed and it influences the behavior
 * - Any of the BLE characteristics permission is changed and it influences the behavior
 * - Internal structure and/or member of packet for any command is added, removed, or changed, which influences the
 * total system behavior
 *
 * Lower byte of spec version should be incremented if any of the following changes is applied.
 * - Internal structure and/or member of packet for any command is added, removed, or changed
 */
#define BLE_PROTOCOL_VERSION (uint16_t)(0x0700)

/**
 * @brief Maximum packet size for BLE communication protocol
 *
 * The packet for BLE communication protocol is constructed with HEADER and BODY. This size includes all of them.
 */
#define NUM_MAX_BLE_PKT (20)

/**
 * @brief Maximum body size on a packet for BLE communication protocol
 *
 * This size is equivalent to the size excluding HEADER of ACMD size (1) and Branch ID size (1) from @ref
 * NUM_MAX_BLE_PKT.
 */
#define NUM_MAX_BLE_PKT_BODY (NUM_MAX_BLE_PKT - 2)

/** @cond */
// ACMD commands
typedef enum {
  ACMD_BRANCHTYPE    = 0x00,
  ACMD_DISCONNECT    = 0x01,
  ACMD_ENCODED_WRITE = 0x02,
  ACMD_ALLDISCONNECT = 0x03,
  ACMD_DCINFO        = 0x04,
  ACMD_BINARY_WRITE  = 0x05,
  ACMD_FIND_BRANCH   = 0x06,
  ACMD_DCMAX         = 0x07,
  ACMD_DCMIN         = 0x08,
  ACMD_UNIQUE_ID     = 0x09,
  ACMD_OTA_STATUS    = 0x0A,
  ACMD_DCINI         = 0x0B,
  ACMD_FIND_CORE     = 0x0C,
  ACMD_BRTYPE        = 0x20,
  ACMD_SHUTDOWN      = 0xF0,
} AppCommand_t;

// Command parameter for ACMD_SHUTDOWN
typedef enum {
  ACMDPARAM_SHUTDOWN_BATTERY_LOW = 0x01,
  ACMDPARAM_SHUTDOWN_SWICTCH_OFF = 0x02,
} AppCommandParamShutdown_t;

// Command parameter for ACMD_FIND_CORE/BRANCH
typedef enum {
  ACMDPARAM_FIND_BLINK_OFF    = 0x00,
  ACMDPARAM_FIND_BLINK_NORMAL = 0x01,
  ACMDPARAM_FIND_BLINK_SLOW   = 0x02,
  ACMDPARAM_FIND_BLINK_RAPID  = 0x03,
} AppCommandParamFind_t;
/** @endcond */

/**
 * @brief UART communication protocol version
 *
 * Higher byte of spec version should be incremented if any of the following changes is applied.
 * - Any of the UART configuration like baudrate is changed and it influences the behavior
 *
 * Lower byte of spec version should be incremented if any of the following changes is applied.
 * - The internal structure and/or member of packet for any command is added, removed, or changedh
 */
#define UART_PROTOCOL_VERSION (uint16_t)(0x0700)

/** @cond */
// DC_INI is supported on this version or later
#define UART_PROTOCOL_VERSION_DCINI (uint16_t)(0x0603)
// No BCMD header on response packet on this version or later
#define UART_PROTOCOL_VERSION_NO_BCMD_ON_RES (uint16_t)(0x0604)
/** @endcond */

/**
 * @brief Baudrate for UART communication protocol
 *
 * This is baudrate of UART to communicate between Core and Branch.
 */
#define UART_PROTOCOL_BAUDRATE (50000)

/**
 * @brief Maximum data size on a packet for UART communication protocol
 *
 * The packet for UART communication protocol is constructed with HEADER (STX + data length), BODY (BCMD + Branch ID +
 * data), and FOOTER (CRC). This size includes BODY which excludes BCMD and Branch ID. This is equivalent to @ref
 * NUM_MAX_BLE_PKT_BODY.
 */
#define NUM_MAX_UART_PKT_BODY_DATA (NUM_MAX_BLE_PKT_BODY)

/**
 * @brief Maximum body size on a packet for UART communication protocol
 *
 * The packet for UART communication protocol is constructed with HEADER, BODY, and FOOTER. This size includes only
 * BODY. This is equivalent to the size appending BCMD size (1) and Branch ID size (1) to @ref
 * NUM_MAX_UART_PKT_BODY_DATA.
 */
#define NUM_MAX_UART_PKT_BODY (NUM_MAX_UART_PKT_BODY_DATA + 2)

/**
 * @brief Maximum packet size for UART communication protocol
 *
 * The packet for UART communication protocol is constructed with HEADER, BODY, and FOOTER. This size includes all of
 * HEADER, BODY, and FOOTER. This is equivalent to the size appending HEADER of STX size (1) and data length size (1),
 * and FOOTER of CRC (1) to @ref NUM_MAX_UART_PKT_BODY.
 */
#define NUM_MAX_UART_PKT (NUM_MAX_UART_PKT_BODY + 3)

/** @cond */
typedef enum {
  STX = 0xC1,
} HeaderType_t;

// Central to Peripherals(Core) and Peripherals(Core) to Branch commands
typedef enum {
  BCMD_DISCOVERY        = 0x00,
  BCMD_ENCODED_WRITE    = ACMD_ENCODED_WRITE,
  BCMD_DCINFO           = ACMD_DCINFO,
  BCMD_BINARY_WRITE     = ACMD_BINARY_WRITE,
  BCMD_FIND_LED         = ACMD_FIND_BRANCH,
  BCMD_READ             = 0x32,
  BCMD_BRANCH_DFU_START = 0x64,
  BCMD_RESET            = 0x99,
  BCMD_INVALID          = 0xFF,
} BranchCommand_t;

// Command parameter for BCMD_READ
typedef enum {
  BCMDPARAM_RESERVED            = 0x00,
  BCMDPARAM_READ_IDENTIFICATION = 0x01,
  BCMDPARAM_READ_ENCODED_DATA   = 0x02,
  BCMDPARAM_READ_DCINFO         = 0x03,
  BCMDPARAM_READ_DCMAX          = 0x04,
  BCMDPARAM_READ_DCMIN          = 0x05,
  BCMDPARAM_READ_DCINI          = 0x06,
} BranchCommandParamRead_t;

// Command parameter for BCMD_FIND_LED
typedef enum {
  BCMDPARAM_FIND_BLINK_OFF    = ACMDPARAM_FIND_BLINK_OFF,
  BCMDPARAM_FIND_BLINK_NORMAL = ACMDPARAM_FIND_BLINK_NORMAL,
  BCMDPARAM_FIND_BLINK_SLOW   = ACMDPARAM_FIND_BLINK_SLOW,
  BCMDPARAM_FIND_BLINK_RAPID  = ACMDPARAM_FIND_BLINK_RAPID,
} BranchCommandParamFind_t;

// Command response type
typedef enum {
  BCMDRES_RESERVED            = 0x00,
  BCMDRES_READ_IDENTIFICATION = BCMDPARAM_READ_IDENTIFICATION,
  BCMDRES_READ_ENCODED_DATA   = BCMDPARAM_READ_ENCODED_DATA,
  BCMDRES_READ_DCINFO         = BCMDPARAM_READ_DCINFO,
  BCMDRES_READ_DCMAX          = BCMDPARAM_READ_DCMAX,
  BCMDRES_READ_DCMIN          = BCMDPARAM_READ_DCMIN,
  BCMDRES_READ_DCINI          = BCMDPARAM_READ_DCINI,
  BCMDRES_DISCOVERY_ACK       = 0xAA,
  BCMDRES_ACK_EMPTY           = 0xBB,
  BCMDRES_NOTHING             = 0xFF,
} BranchCommandRes_t;

// UART communication protocol packet format
typedef struct {
  uint8_t bcmd;
  uint8_t branch_id;
} BcmdPktHdr_t;

typedef struct {
  BcmdPktHdr_t hdr;
  uint8_t      body[NUM_MAX_UART_PKT_BODY_DATA];
} BcmdReqPkt_t;

// TODO: deprecated packet structure
typedef struct {
  BcmdPktHdr_t hdr;
  uint8_t      body[NUM_MAX_UART_PKT_BODY_DATA];
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
/** @endcond */

/*
 * Common communication protocol specification applicable for both App/Core and Core/Branch
 */

/**
 * @brief Maximum number of DC info in a Branch
 */
#define NUM_MAX_DC (16)

/** @cond */
// Maximum data body size in DC ini/min/max packet
#define NUM_MAX_PKT_BODY_DATA_DC     (NUM_MAX_BLE_PKT_BODY - 2)
#define NUM_MAX_PKT_BODY_DATA_DC_INI (NUM_MAX_PKT_BODY_DATA_DC)
#define NUM_MAX_PKT_BODY_DATA_DC_MIN (NUM_MAX_PKT_BODY_DATA_DC)
#define NUM_MAX_PKT_BODY_DATA_DC_MAX (NUM_MAX_PKT_BODY_DATA_DC)
/** @endcond */

/**
 * @brief Branch ID
 *
 * This ID is assigned to the connected branches. ID 1 is for the nearest one to Core. ID 5 is maximum number and
 * farthest from Core.
 */
typedef enum {
  BRANCH_ID_BROADCAST = 0x00,                /**< ID for broadcasting (e.g. used for discovery command) */
  BRANCH_ID_ALL       = BRANCH_ID_BROADCAST, /**< Equivalent to @ref BRANCH_ID_BROADCAST */
  BRANCH_ID_ROOT      = 0x01,                /**< ID for the nearest one to Core */
  BRANCH_ID_2         = 0x02,                /**< ID for second one */
  BRANCH_ID_3         = 0x03,                /**< ID for third one */
  BRANCH_ID_4         = 0x04,                /**< ID for fourth one */
  BRANCH_ID_5         = 0x05,                /**< ID for farthest one */
  BRANCH_ID_MAX       = BRANCH_ID_5,
  BRANCH_ID_IGNORED   = 0xFF,
} BranchId_t;

/**
 * @brief Microcontroller chip types on Branch
 */
typedef enum {
  CHIP_TYPE_ATMEGA_328P  = 0x00, /**< ATmega328P (deprecated) */
  CHIP_TYPE_ATMEGA_328PB = 0x01, /**< ATmega328PB */
} ChipType_t;

#ifdef __cplusplus
}

/**
 * @brief DC groups to be set to @ref dcInfo_t
 */
enum class DcGroup_t : uint8_t {
  DC_GROUP_FOR_SYSTEM = 0, /**< Reserved group for system which does not appear on App */
  DC_GROUP_1,              /**< Group 1 */
  DC_GROUP_2,              /**< Group 2 */
  DC_GROUP_3,              /**< Group 3 */
  DC_GROUP_4,              /**< Group 4 */
  DC_GROUP_5,              /**< Group 5 */
  DC_GROUP_6,              /**< Group 6 */
  DC_GROUP_7,              /**< Group 7 */
  DC_GROUP_8,              /**< Group 8 */
  DC_GROUP_9,              /**< Group 9 */
  DC_GROUP_10,             /**< Group 10 */
  DC_GROUP_11,             /**< Group 11 */
  DC_GROUP_12,             /**< Group 12 */
  DC_GROUP_13,             /**< Group 13 */
  DC_GROUP_14,             /**< Group 14 */
  DC_GROUP_15,             /**< Group 15 */
};

/**
 * @brief DC directions to be set to @ref dcInfo_t
 */
enum class DcNature_t : uint8_t {
  DC_NATURE_OUT = 0, /**< DC for output from Branch Cell */
  DC_NATURE_IN,      /**< DC for input to Branch Cell */
};

/**
 * @brief DC types to be set to @ref dcInfo_t
 */
enum class DcType_t : uint8_t {
  DC_TYPE_BOOLEAN       = 0x01, /**< boolean (Digital as 0 or 1) */
  DC_TYPE_ANALOG_1BYTE  = 0x05, /**< int8_t (Analog as signed 1byte value from -128 to 127) */
  DC_TYPE_ANALOG_2BYTES = 0x06, /**< int16_t (Analog as signed 2bytes value from -32768 to 32767) */
  DC_TYPE_BINARY        = 0x07, /**< uint8_t (Binary as unsigned 1byte values with variable length) */
};
#endif

#endif // COMMUNICATION_PROTOCOL_H
