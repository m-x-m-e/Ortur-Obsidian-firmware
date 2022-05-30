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
#pragma once

/**
 * power.h - power control
 */

#include "../core/millis_t.h"

#define POWER_OFFON_DELAY (10*1000) //millis

class Power {
  public:
	static bool power_on_by_user;
	static bool power_off_by_user;
	static constexpr millis_t power_offon_delay =  POWER_OFFON_DELAY;
    static void check(); //TODO: checke power unit states
    static void power_on(bool by_user = false);
    static void power_off(bool by_user = false,bool kill = false);
  private:
    static millis_t lastPowerOn;
    static millis_t lastPowerOff;
    static bool is_power_needed();
};

extern Power powerManager;
