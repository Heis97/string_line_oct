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

/**
 * parser.cpp - Parser for a G-Code line, providing a parameter interface.
 */

#include "parser.h"

#include "../MarlinCore.h"

// Must be declared for allocation and to satisfy the linker
// Zero values need no initialization.

bool GCodeParser::volumetric_enabled;

#if ENABLED(INCH_MODE_SUPPORT)
  float GCodeParser::linear_unit_factor, GCodeParser::volumetric_unit_factor;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit GCodeParser::input_temp_units = TEMPUNIT_C;
#endif

char *GCodeParser::command_ptr,
     *GCodeParser::string_arg,
     *GCodeParser::value_ptr;
char GCodeParser::command_letter;
uint16_t GCodeParser::codenum;

#if USE_GCODE_SUBCODES
  uint8_t GCodeParser::subcode;
#endif

#if ENABLED(GCODE_MOTION_MODES)
  int16_t GCodeParser::motion_mode_codenum = -1;
  #if USE_GCODE_SUBCODES
    uint8_t GCodeParser::motion_mode_subcode;
  #endif
#endif

#if ENABLED(FASTER_GCODE_PARSER)
  // Optimized Parameters
  uint32_t GCodeParser::codebits;  // found bits
  uint8_t GCodeParser::param[26];  // parameter offsets from command_ptr
#else
  char *GCodeParser::command_args; // start of parameters
#endif

// Create a global instance of the G-Code parser singleton
GCodeParser parser;

/**
 * Clear all code-seen (and value pointers)
 *
 * Since each param is set/cleared on seen codes,
 * this may be optimized by commenting out ZERO(param)
 */
void GCodeParser::reset() {
  string_arg = nullptr;                 // No whole line argument
  command_letter = '?';                 // No command letter
  codenum = 0;                          // No command code
  TERN_(USE_GCODE_SUBCODES, subcode = 0); // No command sub-code
  #if ENABLED(FASTER_GCODE_PARSER)
    codebits = 0;                       // No codes yet
    //ZERO(param);                      // No parameters (should be safe to comment out this line)
  #endif
}

#if ENABLED(GCODE_QUOTED_STRINGS)

  // Pass the address after the first quote (if any)
  char* GCodeParser::unescape_string(char* &src) {
    if (*src == '"') ++src;     // Skip the leading quote
    char * const out = src;     // Start of the string
    char *dst = src;            // Prepare to unescape and terminate
    for (;;) {
      char c = *src++;          // Get the next char
      switch (c) {
        case '\\': c = *src++; break; // Get the escaped char
        case '"' : c = '\0'; break;   // Convert bare quote to nul
      }
      if (!(*dst++ = c)) break; // Copy and break on nul
    }
    return out;
  }

#endif

/**
 * Populate the command line state (command_letter, codenum, subcode, and string_arg)
 * by parsing a single line of G-Code. 58 bytes of SRAM are used to speed up seen/value.
 */
void GCodeParser::parse(char *p) {

  reset(); // No codes to report

  auto uppercase = [](char c) {
    return TERN0(GCODE_CASE_INSENSITIVE, WITHIN(c, 'a', 'z')) ? c + 'A' - 'a' : c;
  };

  // Skip spaces
  while (*p == ' ') ++p;

  // Skip N[-0-9] if included in the command line
  if (uppercase(*p) == 'N' && NUMERIC_SIGNED(p[1])) {
    //TERN_(FASTER_GCODE_PARSER, set('N', p + 1)); // (optional) Set the 'N' parameter value
    p += 2;                  // skip N[-0-9]
    while (NUMERIC(*p)) ++p; // skip [0-9]*
    while (*p == ' ')   ++p; // skip [ ]*
  }

  // *p now points to the current command, which should be G, M, or T
  command_ptr = p;

  // Get the command letter, which must be G, M, or T
  const char letter = uppercase(*p++);

  // Nullify asterisk and trailing whitespace
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;                          // *
    while (*starpos == ' ') --starpos;  // spaces...
    starpos[1] = '\0';
  }

  /**
   * Screen for good command letters. G, M, and T are always accepted.
   * With Motion Modes enabled any axis letter can come first.
   */
  switch (letter) {
    case 'G': case 'M': case 'T': TERN_(MARLIN_DEV_MODE, case 'D':) {
      // Skip spaces to get the numeric part
      while (*p == ' ') p++;


      // Save the command letter at this point
      // A '?' signifies an unknown command
      command_letter = letter;
      // Get the code number - integer digits only
      codenum = 0;

      do { codenum = codenum * 10 + *p++ - '0'; } while (NUMERIC(*p));


      // Skip all spaces to get to the first argument, or nul
      while (*p == ' ') p++;


    } break;


    default: return;
  }

  // The command parameters (if any) start here, for sure!

  IF_DISABLED(FASTER_GCODE_PARSER, command_args = p); // Scan for parameters in seen()


  string_arg = nullptr;
  while (const char param = uppercase(*p++)) {  // Get the next parameter. A NUL ends the loop

    #if ENABLED(FASTER_GCODE_PARSER)
      // Arguments MUST be uppercase for fast G-Code parsing
      #define PARAM_OK(P) WITHIN((P), 'A', 'Z')
    #else
      #define PARAM_OK(P) true
    #endif

    if (PARAM_OK(param)) {

      while (*p == ' ') p++;                    // Skip spaces between parameters & values

      #if ENABLED(GCODE_QUOTED_STRINGS)
        const bool is_str = (*p == '"'), has_val = is_str || valid_float(p);
        char * const valptr = has_val ? is_str ? unescape_string(p) : p : nullptr;
      #else
        const bool has_val = valid_float(p);
        #if ENABLED(FASTER_GCODE_PARSER)
          char * const valptr = has_val ? p : nullptr;
        #endif
      #endif

      //#if ENABLED(DEBUG_GCODE_PARSER)
       // if (debug) {
         // SERIAL_ECHOPGM("Got param ", C(param), " at index ", p - command_ptr - 1);
          //if (has_val) SERIAL_ECHOPGM(" (has_val)");
       // }
      //#endif

      if (!has_val && !string_arg) {            // No value? First time, keep as string_arg
        string_arg = p - 1;
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) SERIAL_ECHOPGM(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
        #endif
      }

      if (TERN0(DEBUG_GCODE_PARSER, debug)) SERIAL_EOL();

      TERN_(FASTER_GCODE_PARSER, set(param, valptr)); // Set parameter exists and pointer (nullptr for no value)
    }
    else if (!string_arg) {                     // Not A-Z? First time, keep as the string_arg
      string_arg = p - 1;
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_ECHOPGM(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
      #endif
    }

    if (!WITHIN(*p, 'A', 'Z')) {                // Another parameter right away?
      while (*p && DECIMAL_SIGNED(*p)) p++;     // Skip over the value section of a parameter
      while (*p == ' ') p++;                    // Skip over all spaces
    }
  }
}


void GCodeParser::unknown_command_warning() {
  SERIAL_ECHO_MSG(STR_UNKNOWN_COMMAND, command_ptr, "\"");
}

#if ENABLED(DEBUG_GCODE_PARSER)

  void GCodeParser::debug() {
    SERIAL_ECHOPGM("Command: ", command_ptr, " (", command_letter);
    SERIAL_ECHO(codenum);
    SERIAL_ECHOLNPGM(")");
    #if ENABLED(FASTER_GCODE_PARSER)
      SERIAL_ECHOPGM(" args: { ");
      for (char c = 'A'; c <= 'Z'; ++c) if (seen(c)) SERIAL_CHAR(c, ' ');
      SERIAL_CHAR('}');
    #else
      SERIAL_ECHOPGM(" args: { ", command_args, " }");
    #endif
    if (string_arg) {
      SERIAL_ECHOPGM(" string: \"", string_arg);
      SERIAL_CHAR('"');
    }
    SERIAL_ECHOLNPGM("\n");
    for (char c = 'A'; c <= 'Z'; ++c) {
      if (seen(c)) {
        SERIAL_ECHOPGM("Code '", c); SERIAL_ECHOPGM("':");
        if (has_value()) {
          SERIAL_ECHOLNPGM(
            "\n    float: ", value_float(),
            "\n     long: ", value_long(),
            "\n    ulong: ", value_ulong(),
            "\n   millis: ", value_millis(),
            "\n   sec-ms: ", value_millis_from_seconds(),
            "\n      int: ", value_int(),
            "\n   ushort: ", value_ushort(),
            "\n     byte: ", value_byte(),
            "\n     bool: ", value_bool(),
            "\n   linear: ", value_linear_units(),
            "\n  celsius: ", value_celsius()
          );
        }
        else
          SERIAL_ECHOLNPGM(" (no value)");
      }
    }
  }

#endif // DEBUG_GCODE_PARSER
