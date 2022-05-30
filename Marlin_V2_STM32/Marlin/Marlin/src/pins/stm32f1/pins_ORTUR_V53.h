/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if !defined(STM32F1) && !defined(STM32F1xx)
  #error "Oops! Select an STM32F4 board in 'Tools > Board.'"
#endif

//#if !defined(__STM32F1__) && !defined(__STM32F4__)
//  #error "Oops! Select an STM32F1/4 board in 'Tools > Board.'"
//#endif

#define DEFAULT_MACHINE_NAME "ORTUR V5.3"
#define BOARD_NAME "Ortur V" STRINGIFY(ORTUR_BOARD)

// Disabling the STM32 IWDG during debugging
#define BOARD_INIT() __HAL_DBGMCU_FREEZE_IWDG()

#define DISABLE_DEBUG
#ifdef DISABLE_DEBUG
  // Release PB3/PB4 PA15 from Jtag pins
  #define JTAG_DISABLE() __HAL_AFIO_REMAP_SWJ_NOJTAG();
  // Release PA13/PA14 from SWD pins
  #ifndef DEBUG
    #define JTAGSWD_DISABLE() __HAL_AFIO_REMAP_SWJ_DISABLE();
  #endif
#endif

#if ENABLED(EEPROM_SETTINGS)
#define FLASH_EEPROM_SPIFLASH
//#define FLASH_EEPROM_EMULATION
//#define I2C_EEPROM

#define E2END 0xFFF // EEPROM end address (4K)

#endif

// Ignore temp readings during develpment.
//#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "STM32F1 supports up to 1 hotends / E-steppers."
#endif

//
// Limit Switches
//
#if ENABLED(DELTA)
	#define X_MAX_PIN          PC13
	#define Y_MAX_PIN          PC14
	#define Z_MAX_PIN          PC15
#else
	#define X_MIN_PIN          PC13
	#define Y_MIN_PIN          PC14
	#define Z_MIN_PIN          PC15
#endif

//
// Z Probe (when not Z_MIN_PIN)
//

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN      PD1
#endif

//
// Steppers
//

#define X_STEP_PIN             PB10
#define X_DIR_PIN              PB11
#define X_ENABLE_PIN           PB2

#define Y_STEP_PIN             PB13
#define Y_DIR_PIN              PB12
#define Y_ENABLE_PIN           PB14

#define Z_STEP_PIN             PB15
#define Z_DIR_PIN              PC8
#define Z_ENABLE_PIN           PC9

#define E0_STEP_PIN            PA9
#define E0_DIR_PIN             PA8
#define E0_ENABLE_PIN          PA10


#ifdef SDSUPPORT
#define SDSS                   PIN_SPI_SS

//TODO:固件正式发布后启用TF卡检测脚
//NOTE:主板没有焊接跳线电阻
//#ifndef DEBUG
//#define SD_DETECT_PIN          PA13
//#endif

// SDCard use Software SPI.
//#define SOFTWARE_SPI
#define CUSTOM_SPI
#endif

//
// Temperature Sensors
//

#define TEMP_0_PIN             PA1   // Analog Input
#define TEMP_BED_PIN           PA0   // Analog Input

//
// Heaters / Fans
//

#define HEATER_0_PIN           PB0
#define HEATER_BED_PIN         PB1

#ifndef FAN_PIN
  #define FAN_PIN              PA7
#endif

#define E0_AUTO_FAN_PIN        PA6

#if ENABLED(USE_CONTROLLER_FAN)
	#if ( !defined(E0_AUTO_FAN_PIN) || E0_AUTO_FAN_PIN == -1) && (!defined(CONTROLLER_FAN_PIN) || CONTROLLER_FAN_PIN == -1)
		#define CONTROLLER_FAN_PIN PA6
	#endif
#endif
//
// Misc. Functions
//

#if ENABLED(PSU_CONTROL)
#define PS_ON_PIN              PA12
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
#define POWER_LOSS_PIN         PA4         //Power loss / nAC_FAULT
#define POWER_LOSS_STATE       LOW
#endif

//
// Filament runout
//

#define FIL_RUNOUT_PIN         PD0

//
// LCD / Controller
//

#define LCD_D0_PIN             PC0
#define LCD_D1_PIN             PC1
#define LCD_D2_PIN             PC2
#define LCD_D3_PIN             PC3
#define LCD_D4_PIN             PC4
#define LCD_D5_PIN             PC5
#define LCD_D6_PIN             PC6
#define LCD_D7_PIN             PC7
#define LCD_CS_PIN             PA11
#define LCD_RS_PIN             PD2
#define LCD_WR_PIN             PC11
#define LCD_RD_PIN             PC12
#define LCD_BACKLIGHT_PIN      PC10  //pwm

#define TOUCH_MOSI_PIN         LCD_D5_PIN
#define TOUCH_MISO_PIN         LCD_D7_PIN
#define TOUCH_SCK_PIN          LCD_D6_PIN
#define TOUCH_CS_PIN           PB8
#define TOUCH_INT_PIN          PB9


#define I2C_SCL_PIN            PIN_WIRE_SCL
#define I2C_SDA_PIN            PIN_WIRE_SDA

//TODO:固件正式发布后启用扬声器
#ifndef DEBUG
  #define BEEPER_PIN           PA14
#endif

#ifdef SERIAL_PORT
	#define SERIAL_RX_PIN      PIN_SERIAL_RX
	#define SERIAL_TX_PIN      PIN_SERIAL_TX
#endif

