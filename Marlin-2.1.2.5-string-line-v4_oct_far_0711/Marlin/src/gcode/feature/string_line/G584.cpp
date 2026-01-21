#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"


//sup_plate
void GcodeSuite::G584() {
  //------------------X vibro-----------------------
  //if (parser.seen('X'))                      {string_manager.string_vibro = parser.intval('X');};
  //if (parser.seen('X') && parser.seen('V'))  {string_manager.vibr_x = parser.intval('V');}; 
  //if (parser.seen('X') && parser.seen('D'))  {string_manager.dir_x = parser.intval('D');};
  //if (parser.seen('X') && parser.seen('W'))  {string_manager.vibr_a_x = parser.intval('W');};

//-------------------Y-----------------------
 /*if (parser.seen('Y'))                      {string_manager.gateway_move = parser.intval('Y');};
 if (parser.seen('Y') && parser.seen('D'))  {string_manager.dir_y = parser.intval('D');};
//-------------------Z-----------------------
 if (parser.seen('Z'))                      {string_manager.recuperator_move = parser.intval('Z');};
 if (parser.seen('Z') && parser.seen('D'))  {string_manager.dir_z = parser.intval('D');};
//------------------turbo----------------------------------------------------------------------
  if (parser.seen('F')) 
  {
        string_manager.set_turbo(parser.floatval('F'));
  } 


  */
}
