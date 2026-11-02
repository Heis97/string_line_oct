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

// R25 = 100 kOhm, beta25 = 4092 K, 4.7 kOhm pull-up, bed thermistor
constexpr temp_entry_t temptable_1[] PROGMEM = {
  { OV(  18), 320 },
  { OV(  19), 315 },
  { OV(  20), 310 },
  { OV(  22), 305 },
  { OV(  23), 300 },
  { OV(  25), 295 },
  { OV(  27), 260 },
  { OV(  28), 255 },
  { OV(  31), 240 },
  { OV(  33), 235 },
  { OV(  35), 230 },
  { OV(  38), 225 },
  { OV(  41), 220 },


  { OV( 320),  164 },
  { OV( 379),  151 },
  { OV( 452),  140 },
  { OV( 509),  129 },
  { OV( 598),  114 },
  { OV( 665),  104 },//90
  { OV( 977),  25 },
  { OV( 985),  20 },
  { OV( 993),  15 },
  { OV( 999),  10 },
  { OV(1004),   5 },
  { OV(1008),   0 },
  { OV(1012),  -5 },
  { OV(1016), -10 },
  { OV(1020), -15 }
};
