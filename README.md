# What's this

Library to talk with VIVIWARE Cell Core.

VivicoreSerial library works on ATmega328PB which has 2 UART ports. One is for talking with VIVIWARE Cell Core, another is for flashing and debugging.

The official sketches of VIVIWARE Cell Branch are included as examples code.

# Requisites

The below table shows the corresponding Arduino IDE version VivicoreSerial library requires. Refer to [How to setup](#how-to-setup) to install Arduino IDE and configure the board and library manager.

| Arduino IDE version | VivicoreSerial version managed on Library Manager | `LIBRARY_VER_BUILD_NO` defined on `VivicoreSerialVersion.h` | Description |
|-|-|-|-|
| 1.8.12 (Required)   | 1.0.0 or later         | 0x0005 or later      |<ul><li>64bit compatibility with upcoming macOS Catalina</li><li>avr-gcc 7.3.0-atmel3.6.1-arduino5</li></ul>|
| 1.8.7               | -                      | 0x0004 or before     |<ul><li>avr-gcc 5.4.0</li></ul>|

# Dependencies and lisence information

VivicoreSerial library depends on Arduino core and libraries, and ATmegaBOOT.

And a part of the official sketches of VIVIWARE Cell Branch included as examples code depends on the following version's library modules. The library modules are neccessary to be installed on library manager or with archived zip file on Github if building examples code.

|Module|Lisence and the link to the original|
|-|-|
|[Adafruit_APDS9960 v1.1.4](https://github.com/adafruit/Adafruit_APDS9960/releases/tag/1.1.4)|[BSD-3-Clause](https://github.com/adafruit/Adafruit_APDS9960/blob/master/license.txt)|
|[vl53l0x-arduino v1.0.2](https://github.com/pololu/vl53l0x-arduino/releases/tag/1.0.2)|[MIT (pololu), BSD-3-Clause (STMicroelectronics)](https://github.com/pololu/vl53l0x-arduino/blob/master/LICENSE.txt)|
|[Arduino-LSM6DS3-LSM6DS3TRC v1.0.0](https://github.com/vivitainc/Arduino-LSM6DS3-LSM6DS3TRC/releases/tag/1.0.0)|[BSD-3-Clause](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/LICENSE)|
|[Arduino-misakiUTF16 v1.02a](https://github.com/vivitainc/Arduino-misakiUTF16/releases/tag/1.02a)|[Free software lisence](https://github.com/Tamakichi/Arduino-misakiUTF16)<br>本フォントライブラリは、「美咲フォント」と同様にフリー（自由な）ソフトウエアです。<br>あらゆる改変の有無に関わらず、また商業的な利用であっても、自由にご利用、複製、再配布することができます。<br>ただし、全て無保証とさせていただきます。|
|Misaki font (included in Arduino-misakiUTF16)|[Free software lisence](https://littlelimit.net/font.htm#license)<br>These fonts are free software.<br>Unlimited permission is granted to use, copy, and distribute them, with or without modification, either commercially or noncommercially.<br>THESE FONTS ARE PROVIDED "AS IS" WITHOUT WARRANTY.|
|Arduino core and libraries (included in Arduino IDE)|[LGPLv2.1](https://github.com/arduino/Arduino/blob/master/license.txt)|
|[ATmegaBOOT](https://github.com/vivitainc/328pb_bootloader/blob/develop/bootloaders/atmega/ATmegaBOOT_168.c) (included in board package setup on [How to setup](#how-to-setup))|[GPLv2](https://github.com/arduino/ArduinoCore-avr/blob/master/bootloaders/atmega/ATmegaBOOT_168.c)|

**NOTE:**
The following libraries cannot be found on Library Manager, and need to be installed by zip archive got from the above Github link if you build and upload the examples code depending them.
- Arduino-misakiUTF16
- Arduino-LSM6DS3-LSM6DS3TRC

# How to setup

This instruction describes how to setup Arduino IDE to build VIVIWARE Cell Custom sketch and upload it to the board.

## Install and configure Arduino IDE

1. Download and install the required Arduino IDE version written on [requisites](#requisites) which is available on [Arduino official site top](https://www.arduino.cc/en/Main/Software) or [Previous IDE Releases](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous).
    - For Windows, **use Windows Installer exe file but not zip file**
    - On Board Manager in Arduino IDE, **do not update built-in packages (e.g. Arduino AVR Boards)** to avoid unexpected built result mismatch. Use default build-in packages bundled with the required Arduino IDE version.
2. Open `Preferences` of Arduino IDE.
3. Add URL `https://raw.githubusercontent.com/vivitainc/custom_cell_boards/master/package_vivita_index.json` into `Additional Boards Manager URLs` text box, and press `OK` button.

## Setup Boards Manager

1. Open Arduino IDE and the menu `Tools` > `Board:` > `Boards Manager...`.
2. Select `Type` of `All` or `Contributed`, enter `viviware` into text box, and find `VIVIWARE Cell Custom Boards`.
3. Select the latest version of `VIVIWARE Cell Custom Boards` (e.g. `1.0.1`), and press `Install` button.
4. After a while, check the status `INSTALLED` shown on right side of the version.
5. Close `Boards Manager` by pressing `Close` button.

## Setup Library Manager

1. Open Arduino IDE and the menu `Sketch` > `Include Library` > `Manage Libraries...`.
2. Select `Type` of `All` or `Contributed`, select `Topic` of `All` or `Communication`, and enter `VivicoreSerial` into text box, and find `VivicoreSerial` library.
3. Select the latest version of `VivicoreSerial` (e.g. `1.0.0`), and press `Install` button.
4. After a while, check the status `INSTALLED` shown on right side of the version.
5. Close `Library Manager` by pressing `Close` button.

## Build and upload sketch

1. Open new sketch on Arduino IDE.
2. Copy the template code generated by VIVIWARE Developer and paste it to the new sketch.
3. Select board `VIVIWARE Cell` in the menu `Tools` > `Board:`.
4. Select version `Custom` in the menu `Tools` > `Version:`.
5. Select port connected to VIVIWARE Cell Custom board in the menu `Tools` > `Port:`.
6. Press `Upload` button.

# Only for VivicoreSerial library developer

Refer to [README](https://github.com/vivitainc/branch_cell/blob/develop/README.md) for library developer.
