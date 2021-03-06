/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(POWER_LOSS_RECOVERY)

#include "../../gcode.h"
#include "../../../feature/powerloss.h"
#include "../../../module/motion.h"
#include "../../../lcd/ultralcd.h"

/**
 * M413: Enable / Disable power-loss recovery
 *
 * Parameters
 *   S[bool] - Flag to enable / disable.
 *             If omitted, report current state.
 */
void GcodeSuite::M413() {

  if (parser.seen('S'))
    recovery.enable(parser.value_bool());
  else {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Power-loss recovery ");
    serialprintln_onoff(recovery.enabled);
  }

  #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    if (parser.seen("RL")) recovery.load();
    if (parser.seen('W')){
      #if POWER_LOSS_ZRAISE
        // Get the limited Z-raise to do now or on resume
        const float zraise = _MAX(0, _MIN(current_position.z + POWER_LOSS_ZRAISE, Z_MAX_POS - 1) - current_position.z);
      #else
        constexpr float zraise = 0;
      #endif
      recovery.save(true,zraise);
    }
    if (parser.seen('P')) recovery.purge();
    #if PIN_EXISTS(POWER_LOSS)
      if (parser.seen('O')) recovery._outage();
    #endif
    if (parser.seen('E')) serialprintPGM(recovery.exists() ? PSTR("PLR Exists\n") : PSTR("No PLR\n"));
    if (parser.seen('V')) serialprintPGM(recovery.valid() ? PSTR("Valid\n") : PSTR("Invalid\n"));
  #endif
}

#endif // POWER_LOSS_RECOVERY
