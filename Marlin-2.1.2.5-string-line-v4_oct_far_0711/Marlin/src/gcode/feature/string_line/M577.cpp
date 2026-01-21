#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M577() {
  #ifdef PRIMARY_PLATE
  if (parser.seen('V')) 
  {
    string_manager.set_hv_v(parser.intval('V'));
  } 
  if (parser.seen('I')) 
  {
    string_manager.set_hv_i(parser.intval('I'));
  } 
  if (parser.seen('G')) 
  {
    string_manager.get_v_hv();
  } 
  #endif
}
