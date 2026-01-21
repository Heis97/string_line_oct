#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

//#include "../../../MarlinCore.h"

void GcodeSuite::M578() {
  #ifdef PRIMARY_PLATE
  if (parser.seen('S')) 
  {
    string_manager.set_press(parser.intval('S'));
  } 
  if (parser.seen('G')) 
  {
    string_manager.get_v_press();
  } 

  if (parser.seen('U')) 
  {
    //MYSERIAL2.println("N1 G91");
    //MYSERIAL2.println("N2 G1 U10 F600");
  } 

  if (parser.seen('F')) 
  {
    string_manager.motors_free(parser.intval('F'));
  } 

  if (parser.seen('A')) 
  {
    string_manager.tare_tens(parser.intval('A'));
  } 

  
  #endif
}
