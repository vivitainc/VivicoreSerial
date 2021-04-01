/*
  VivicoreSerialVersion.h - Library version definition for VIVIWARE Cell Branch
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

#ifndef VIVICORE_SERIAL_VERSION_H
#define VIVICORE_SERIAL_VERSION_H

#define LIBRARY_VER_BUILD_NO (0x000E)

#if defined(MIN_LIBRARY_VER_BUILD_NO)
#  if MIN_LIBRARY_VER_BUILD_NO > LIBRARY_VER_BUILD_NO
#    error "Your ino code requires later version's VivicoreSerial library than your using one. Please update the library."
#  endif
#endif

#endif
