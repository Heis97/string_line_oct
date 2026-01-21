#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"


//STRING CONTROL
void GcodeSuite::M584() {
  #ifdef PRIMARY_PLATE
  //string sup--------------------------------------
  if (parser.seen('I'))
  {
    if(parser.seen('O'))  string_manager.force_off[parser.intval('I')] = parser.floatval('O'); 
    if (parser.seen('P'))  string_manager.force_k[parser.intval('I')] = parser.floatval('P');
    if (parser.seen('K'))  string_manager.koef_v_tens[parser.intval('I')] = parser.floatval('K');
    if (parser.seen('Z'))  string_manager.orig_speed_tens[parser.intval('I')]  = parser.floatval('Z');//Z
    
    if (parser.seen('J') && parser.seen('L') ) 
    {
      float val = parser.floatval('J');
      for(int i=0; i<TENSOMETR_NUM; i++) string_manager.force_dest[i]  = val;
      //Serial.println(val);
    }
    else if (parser.seen('J')) 
    {
         string_manager.force_dest[parser.intval('I')]  = parser.floatval('J');//J
    }
    if (parser.seen('W'))  string_manager.dir[string_manager.motors_tens[parser.intval('I')]]  = parser.intval('W');//W
    if (parser.seen('H'))  string_manager.koef_gauss[parser.intval('I')]  = parser.floatval('H');//Z
  }
  else
  {
    if (parser.seen('A'))  string_manager.string_move_second[0]  = parser.intval('A');
    if (parser.seen('B'))  string_manager.string_move_second[1]  = parser.intval('B');
    if (parser.seen('C'))  string_manager.string_move_second[2]  = parser.intval('C');
    if (parser.seen('D'))  string_manager.string_move_second[3]  = parser.intval('D');
    if (parser.seen('F'))  string_manager.string_move_second[4]  = parser.intval('F');
    //string main--------------------------------------


    if (parser.seen('U'))  string_manager.string_move  = parser.intval('U');
  // if (parser.seen('U') && parser.seen('V'))  string_manager.speed_koef  = parser.floatval('V');
    if (parser.seen('U') && parser.seen('Z'))  string_manager.orig_speed_tens_com  = parser.floatval('W');
    if (parser.seen('U') && parser.seen('W'))  string_manager.dir[string_manager.motor_com_axis]  = parser.intval('W');


        if (parser.seen('U') && parser.seen('E'))  string_manager.set_vel_strings(parser.floatval('E'));
         if (parser.seen('U') && parser.seen('O'))  string_manager.reset_vel_strings();


  }
  
  #endif
  //-------------------------------------


}
