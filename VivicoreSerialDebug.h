/*
  Copyright (c) 2021 VIVIWARE JAPAN, Inc.  All right reserved.

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
 * @file VivicoreSerialDebug.h
 * @brief Debug print library for VIVIWARE Cell Branch and Custom
 */

#ifndef VIVICORESERIAL_DEBUG_H
#define VIVICORESERIAL_DEBUG_H

//#define DBG_GPIO_ENABLED
//#define DBG_SERIAL_ENABLED
//#define DISABLE_VIVICORE

#if defined(DBG_SERIAL_ENABLED)
#  define SERIAL_DEBUG0_ENABLED
//#define SERIAL_DEBUG1_ENABLED
//#define SERIAL_DEBUG2_ENABLED
//#define SERIAL_DEBUG_CIBO_ENABLED
//#define SERIAL_DEBUG_COBI_ENABLED
#endif

// GPIO Debug Macro definition
#if defined(DBG_GPIO_ENABLED)
#  define DebugGPIODirectOut(DBG_GPIO_DDR, DBG_GPIO_BIT) DBG_GPIO_DDR = DBG_GPIO_DDR | (1 << DBG_GPIO_BIT)
#  define DebugGPIOLow(DBG_GPIO_PORT, DBG_GPIO_BIT)      DBG_GPIO_PORT = DBG_GPIO_PORT & ~(1 << DBG_GPIO_BIT)
#  define DebugGPIOHigh(DBG_GPIO_PORT, DBG_GPIO_BIT)     DBG_GPIO_PORT = DBG_GPIO_PORT | (1 << DBG_GPIO_BIT)
#  define DebugGPIOWrite(DBG_GPIO_PORT, DBG_GPIO_BIT, BIT_VALUE) \
    (BIT_VALUE ? DebugGPIOHigh(DBG_GPIO_PORT, DBG_GPIO_BIT) : DebugGPIOLow(DBG_GPIO_PORT, DBG_GPIO_BIT))
#else
#  define DebugGPIODirectOut(DBG_GPIO_DDR, DBG_GPIO_BIT)
#  define DebugGPIOLow(DBG_GPIO_PORT, DBG_GPIO_BIT)
#  define DebugGPIOHigh(DBG_GPIO_PORT, DBG_GPIO_BIT)
#  define DebugGPIOWrite(DBG_GPIO_PORT, DBG_GPIO_BIT, BIT_VALUE)
#endif

// Debug Macro definition

// Debug Serial Instance definition
#define HW_SERIAL_BAUD (250000)
#if (CORE_COMM_UART_PORT == 0)
#  define DBG_SERIAL_INSTANCE Serial1
#elif (CORE_COMM_UART_PORT == 1)
#  define DBG_SERIAL_INSTANCE Serial
#else
#  error Not supported!
#endif

// begin definition
#if defined(DBG_SERIAL_ENABLED)
#  define DebugBegin() DBG_SERIAL_INSTANCE.begin(HW_SERIAL_BAUD)
#else
#  define DebugBegin()
#endif

// print definitions
#if defined(DBG_SERIAL_ENABLED)
#  define DebugFlush(...) DBG_SERIAL_INSTANCE.flush()
#  define DebugStackPointerPrint(...) \
    DBG_SERIAL_INSTANCE.print("SP:"); \
    DBG_SERIAL_INSTANCE.print((int)SP, HEX); \
    DBG_SERIAL_INSTANCE.println()
#else
#  define DebugFlush(...)
#  define DebugStackPointerPrint(...)
#endif
#define DebugPrint(...) \
  DBG_SERIAL_INSTANCE.print('['); \
  DBG_SERIAL_INSTANCE.print(millis()); \
  DBG_SERIAL_INSTANCE.print("] "); \
  DBG_SERIAL_INSTANCE.print(__PRETTY_FUNCTION__); \
  DBG_SERIAL_INSTANCE.print(' '); \
  DBG_SERIAL_INSTANCE.print(__LINE__); \
  DBG_SERIAL_INSTANCE.print(": "); \
  DBG_SERIAL_INSTANCE.print(__VA_ARGS__)
#define DebugPrintln(...) \
  DBG_SERIAL_INSTANCE.print('['); \
  DBG_SERIAL_INSTANCE.print(millis()); \
  DBG_SERIAL_INSTANCE.print("] "); \
  DBG_SERIAL_INSTANCE.print(__PRETTY_FUNCTION__); \
  DBG_SERIAL_INSTANCE.print(' '); \
  DBG_SERIAL_INSTANCE.print(__LINE__); \
  DBG_SERIAL_INSTANCE.print(": "); \
  DBG_SERIAL_INSTANCE.println(__VA_ARGS__)
#define DebugPlainPrint(...)   DBG_SERIAL_INSTANCE.print(__VA_ARGS__)
#define DebugPlainPrintln(...) DBG_SERIAL_INSTANCE.println(__VA_ARGS__)

#define Debug1ByteHexPrint(v) \
  if ((((v) >> 4) & 0x0F) > 9) { \
    DBG_SERIAL_INSTANCE.print((char)('A' + ((uint8_t)(((v) >> 4) & 0x0F) - 10))); \
  } else { \
    DBG_SERIAL_INSTANCE.print((uint8_t)(((v) >> 4) & 0x0F)); \
  } \
  if (((v)&0x0F) > 9) { \
    DBG_SERIAL_INSTANCE.print((char)('A' + ((uint8_t)((v)&0x0F) - 10))); \
  } else { \
    DBG_SERIAL_INSTANCE.print((uint8_t)((v)&0x0F)); \
  }

#define DebugHexPrint(v) Debug1ByteHexPrint(v)

#define DebugBinPrint(v) \
  { \
    for (uint32_t _bit = 1UL << ((sizeof(v) * 8) - 1); _bit; _bit >>= 1) { \
      DBG_SERIAL_INSTANCE.print(v &_bit ? '1' : '0'); \
    } \
  }

#if defined(SERIAL_DEBUG0_ENABLED)
#  define DebugPrint0(...)         DebugPrint(__VA_ARGS__)
#  define DebugPrintln0(...)       DebugPrintln(__VA_ARGS__)
#  define DebugPlainPrint0(...)    DebugPlainPrint(__VA_ARGS__)
#  define DebugPlainPrintln0(...)  DebugPlainPrintln(__VA_ARGS__)
#  define DebugHexPrint0(v)        DebugHexPrint(v)
#  define DebugBinPrint0(v)        DebugBinPrint(v)
#  define DebugStringPrint0(str)   DebugPlainPrint(F(str))
#  define DebugStringPrintln0(str) DebugPlainPrintln(F(str))
#else
#  define DebugPrint0(...)
#  define DebugPrintln0(...)
#  define DebugPlainPrint0(...)
#  define DebugPlainPrintln0(...)
#  define DebugHexPrint0(v)
#  define DebugBinPrint0(v)
#  define DebugStringPrint0(str)
#  define DebugStringPrintln0(str)
#endif

#if defined(SERIAL_DEBUG1_ENABLED)
#  define DebugPrint1(...)         DebugPrint(__VA_ARGS__)
#  define DebugPrintln1(...)       DebugPrintln(__VA_ARGS__)
#  define DebugPlainPrint1(...)    DebugPlainPrint(__VA_ARGS__)
#  define DebugPlainPrintln1(...)  DebugPlainPrintln(__VA_ARGS__)
#  define DebugHexPrint1(v)        DebugHexPrint(v)
#  define DebugBinPrint1(v)        DebugBinPrint(v)
#  define DebugStringPrint1(str)   DebugPlainPrint(F(str))
#  define DebugStringPrintln1(str) DebugPlainPrintln(F(str))
#else
#  define DebugPrint1(...)
#  define DebugPrintln1(...)
#  define DebugPlainPrint1(...)
#  define DebugPlainPrintln1(...)
#  define DebugHexPrint1(v)
#  define DebugBinPrint1(v)
#  define DebugStringPrint1(str)
#  define DebugStringPrintln1(str)
#endif

#if defined(SERIAL_DEBUG2_ENABLED)
#  define DebugPrint2(...)         DebugPrint(__VA_ARGS__)
#  define DebugPrintln2(...)       DebugPrintln(__VA_ARGS__)
#  define DebugPlainPrint2(...)    DebugPlainPrint(__VA_ARGS__)
#  define DebugPlainPrintln2(...)  DebugPlainPrintln(__VA_ARGS__)
#  define DebugHexPrint2(v)        DebugHexPrint(v)
#  define DebugBinPrint2(v)        DebugBinPrint(v)
#  define DebugStringPrint2(str)   DebugPlainPrint(F(str))
#  define DebugStringPrintln2(str) DebugPlainPrintln(F(str))
#else
#  define DebugPrint2(...)
#  define DebugPrintln2(...)
#  define DebugPlainPrint2(...)
#  define DebugPlainPrintln2(...)
#  define DebugHexPrint2(v)
#  define DebugBinPrint2(v)
#  define DebugStringPrint2(str)
#  define DebugStringPrintln2(str)
#endif

#if defined(SERIAL_DEBUG_CIBO_ENABLED)
#  define DebugPrintCIBO(str)   DebugPlainPrint(F(str))
#  define DebugPrintCIBOln(str) DebugPlainPrintln(F(str))
#  define DebugHexPrintCIBO(v)  DebugHexPrint(v)
#else
#  define DebugPrintCIBO(str)
#  define DebugPrintCIBOln(str)
#  define DebugHexPrintCIBO(v)
#endif

#if defined(SERIAL_DEBUG_COBI_ENABLED)
#  define DebugPrintCOBI(str)   DebugPlainPrint(F(str))
#  define DebugPrintCOBIln(str) DebugPlainPrintln(F(str))
#  define DebugHexPrintCOBI(v)  DebugHexPrint(v)
#else
#  define DebugPrintCOBI(str)
#  define DebugPrintCOBIln(str)
#  define DebugHexPrintCOBI(v)
#endif

#endif
