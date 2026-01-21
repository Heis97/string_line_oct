//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"



void GcodeSuite::M585() {





  #ifndef PRIMARY_PLATE
  //BUNKER CONTROL
      //------------------X vibro-----------------------
  if (parser.seen('X'))                      {string_manager.feed_pound_move = parser.intval('X');};
   if (parser.seen('X') && parser.seen('V'))  {motors._vibro[string_manager.feed_pound_axis] = parser.intval('V');}; 
  if (parser.seen('X') && parser.seen('D'))  {motors.vibro_ampl[string_manager.feed_pound_axis] = parser.intval('D');};

 
  //if (parser.seen('X') && parser.seen('W'))  {string_manager.k_m[string_manager.feed_pound_axis]= parser.intval('W');};



//-------------------Y-----------------------
  if (parser.seen('Y'))                      {string_manager.gateway_move = parser.intval('Y');};
   if (parser.seen('Y') && parser.seen('V'))  {motors._vibro[string_manager.gateway_axis] = parser.intval('V');}; 
  if (parser.seen('Y') && parser.seen('D'))  {motors.vibro_ampl[string_manager.gateway_axis] = parser.intval('D');};

  if ( parser.seen('W'))  {string_manager.vibro_vel = parser.intval('W');};
 // if (parser.seen('X') && parser.seen('W'))  {string_manager.k_m[string_manager.feed_pound_axis]= parser.intval('W');};

//----------------------------------------------------------------------

  if (parser.seen('Y') && parser.seen('U'))  {string_manager.vibro_time = parser.intval('U');};
  if (parser.seen('Y') && parser.seen('K'))  {string_manager.relax_time = parser.intval('K');};

 // if (parser.seen('Y') && parser.seen('S'))  {string_manager.s_y_def = parser.floatval('S');};
 // if (parser.seen('Y') && parser.seen('O'))  {string_manager.s_y_vibr = parser.floatval('O');};

  //-------------------Z-----------------------
  if (parser.seen('Z'))                      {string_manager.recuperator_move = parser.intval('Z');};
  if (parser.seen('Z') && parser.seen('D'))  {string_manager.dir[string_manager.recuperator_axis] = parser.intval('D');};


//------------------turbo----------------------------------------------------------------------
  if (parser.seen('F')) 
  {
    string_manager.set_turbo(parser.floatval('F'));
    
  } 

   //MICROSCOP CONTROL
//---------------------microsc D-----AB---------------
if (parser.seen('A') && parser.seen('H'))  {string_manager.move_betw_string_d(parser.floatval('H'));} ;
if (parser.seen('A') && parser.seen('V'))  {string_manager.move_depth_d(parser.floatval('V'));} ;
if (parser.seen('A') && parser.seen('P'))  {string_manager.move_to_pos_d(parser.intval('P'));} ;
if (parser.seen('A') && parser.seen('G'))  {string_manager.home_d(parser.intval('G'));} ;

if (parser.seen('A') && parser.seen('I') && parser.seen('S'))  string_manager.pos_camera_d[parser.intval('I')] = parser.floatval('S');
if (parser.seen('A') && parser.seen('I') && parser.seen('L'))  string_manager.pos_mirror_d[parser.intval('I')] = parser.floatval('L');

if (parser.seen('A') && parser.seen('R'))  string_manager.set_led_micro_d(parser.intval('R'));


//---------------------microsc E-----CU---------------
if (parser.seen('C') && parser.seen('H'))  {string_manager.move_betw_string_e(parser.floatval('H'));} ;
if (parser.seen('C') && parser.seen('V'))  {string_manager.move_depth_e(parser.floatval('V'));} ;
if (parser.seen('C') && parser.seen('P'))  {string_manager.move_to_pos_e(parser.intval('P'));} ;
if (parser.seen('C') && parser.seen('G'))  {string_manager.home_e(parser.intval('G'));} ;

if (parser.seen('C') && parser.seen('I') && parser.seen('S'))  string_manager.pos_camera_e[parser.intval('I')] = parser.floatval('S');
if (parser.seen('C') && parser.seen('I') && parser.seen('L'))  string_manager.pos_mirror_e[parser.intval('I')] = parser.floatval('L');

if (parser.seen('C') && parser.seen('R'))  string_manager.set_led_micro_e(parser.intval('R'));



#endif
}
