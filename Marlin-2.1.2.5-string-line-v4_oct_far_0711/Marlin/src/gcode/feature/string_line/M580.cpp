#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

//main_plate
void GcodeSuite::M580() {

#ifdef PRIMARY_PLATE
//-------------------Z-----------------------
 if (parser.seen('Z'))                      string_manager.karet_move = parser.intval('Z');
 if (parser.seen('Z') && parser.seen('D'))  string_manager.dir[string_manager.karet_axis] = parser.intval('D');
 #endif
}
