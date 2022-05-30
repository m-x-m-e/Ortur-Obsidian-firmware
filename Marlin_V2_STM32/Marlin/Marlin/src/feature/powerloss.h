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
 * power_loss_recovery.h - Resume an SD print after power-loss
 */

#include "../sd/cardreader.h"
#include "../gcode/gcode.h"

#include "../inc/MarlinConfig.h"

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if !defined(POWER_LOSS_STATE) && PIN_EXISTS(POWER_LOSS)
  #define POWER_LOSS_STATE HIGH
#endif

//#define DEBUG_POWER_LOSS_RECOVERY

#ifndef POWER_LOSS_THRESHOLD
  #define POWER_LOSS_THRESHOLD 2000
#endif

#ifndef POWER_LOSS_INTERVAL
  #define POWER_LOSS_INTERVAL 1  //1 millisecond
#endif

//#define SAVE_EACH_CMD_MODE

#if ORTUR_BOARD < 60
#define SAVE_INFO_INTERVAL_MS (10*1000)
#elif ORTUR_BOARD >= 60
#define SAVE_INFO_INTERVAL_MS (30*1000)
#endif

typedef struct {
  uint8_t valid_head;

  // Machine state
  xyze_pos_t current_position;
  bool print_paused;
  xyze_pos_t resume_position;
  float zraise;

  #if HAS_HOME_OFFSET
    xyz_pos_t home_offset;
  #endif
  #if HAS_POSITION_SHIFT
    xyz_pos_t position_shift;
  #endif

  uint16_t feedrate;

  #if EXTRUDERS > 1
    uint8_t active_extruder;
  #endif

  #if DISABLED(NO_VOLUMETRICS)
    bool volumetric_enabled;
    float filament_size[EXTRUDERS];
  #endif

  #if HAS_HOTEND
    int16_t target_temperature[HOTENDS];
  #endif

  #if HAS_HEATED_BED
    int16_t target_temperature_bed;
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    bool power_loss_recovery_active;
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    bool filament_runout_active;
  #endif

  #if HAS_FAN
    uint8_t fan_speed[FAN_COUNT];
  #endif

  #if HAS_LEVELING
    bool leveling;
    float fade;
  #endif

  #if ENABLED(FWRETRACT)
    float retract[EXTRUDERS], retract_hop;
  #endif

  // Mixing extruder and gradient
  #if ENABLED(MIXING_EXTRUDER)
    //uint_fast8_t selected_vtool;
    //mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
    #if ENABLED(GRADIENT_MIX)
      gradient_t gradient;
    #endif
  #endif

  // Relative axis modes
  uint8_t axis_relative;

  // Feedrate Percentage
  int16_t feedrate_percentage;

  // Flow Percentage
#if EXTRUDERS
  int16_t flow_percentage[EXTRUDERS];
#endif

  // SD Filename and position
  char sd_filename[MAXPATHNAMELENGTH];
  volatile uint32_t sdpos;

  // Job elapsed time
  millis_t print_job_elapsed;

  uint8_t valid_foot;

  bool valid() { return valid_head && valid_head == valid_foot; }

} job_recovery_info_t;

class PrintJobRecovery {
  public:
    static const char filename[5];

    static SdFile file;
    static job_recovery_info_t info;

    static uint8_t queue_index_r;     //!< Queue index of the active command
    static uint32_t cmd_sdpos,        //!< SD position of the next command
                    sdpos[BUFSIZE];   //!< SD positions of queued commands

    #if ENABLED(DWIN_CREALITY_LCD)
      static bool dwin_flag;
    #endif

    static constexpr uint32_t power_loss_threshold = POWER_LOSS_THRESHOLD;
    static uint32_t power_loss_count;
    static constexpr uint32_t power_loss_interval = POWER_LOSS_INTERVAL;
    static uint32_t power_loss_milli;

    static void init();
    static void prepare();

    static inline void setup() {
      #if PIN_EXISTS(POWER_LOSS)
        #if ENABLED(POWER_LOSS_PULL)
          #if POWER_LOSS_STATE == LOW
            SET_INPUT_PULLUP(POWER_LOSS_PIN);
          #else
            SET_INPUT_PULLDOWN(POWER_LOSS_PIN);
          #endif
        #else
          SET_INPUT(POWER_LOSS_PIN);
        #endif
      #endif
    }

    // Track each command's file offsets
    static inline uint32_t command_sdpos() { return sdpos[queue_index_r]; }
    static inline void commit_sdpos(const uint8_t index_w) { sdpos[index_w] = cmd_sdpos; }

    static bool enabled;
    static void enable(const bool onoff);
    static void changed();

    static inline bool exists() { return card.jobRecoverFileExists(); }
    static inline void open(const bool read) { card.openJobRecoveryFile(read); }
    static inline void close() { file.close(); }

    static void check();
    static void resume();
    static void purge();

    static inline void cancel() { purge(); card.autostart_index = 0; }

    static void load();
    static void save(const bool force=ENABLED(SAVE_EACH_CMD_MODE), const float zraise=0);

    #if PIN_EXISTS(POWER_LOSS)
      static inline void outage() {
      if (enabled)
      {
		//NOTE: 过滤一下断电信号
		if(READ(POWER_LOSS_PIN) == POWER_LOSS_STATE )
		{
			if(power_loss_count > 0
			    && millis() >= power_loss_milli)
			{
				--power_loss_count;
				power_loss_milli = millis() + power_loss_interval;
			}
		}
		else
		{
			power_loss_count = power_loss_threshold;
			power_loss_milli = millis();
		}

		_outage(!power_loss_count);
      }
    }
    #endif

    static inline bool valid() { return info.valid(); }

    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      static void debug(PGM_P const prefix);
    #else
      static inline void debug(PGM_P const) {}
    #endif

  private:
    static void write();

    #if ENABLED(BACKUP_POWER_SUPPLY)
      static void retract_and_lift(const float &zraise);
    #endif

    #if PIN_EXISTS(POWER_LOSS)
      friend class GcodeSuite;
      static void _outage(bool power_loss_trigged = true);
    #endif
};

extern PrintJobRecovery recovery;
