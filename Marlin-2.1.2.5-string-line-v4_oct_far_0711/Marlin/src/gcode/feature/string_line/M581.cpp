#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"


void GcodeSuite::M581() {
      #ifndef PRIMARY_PLATE
  if (parser.seen('X')) 
  {
       // string_manager.k_m_x = parser.intval('X');
  } 

   if (parser.seen('D')) 
  {
        string_manager.dist_m = parser.floatval('D');
  } 

   if (parser.seen('L')) 
  {
        string_manager.buff_m = parser.intval('L');
  } 



 if (parser.seen('A')) 
  {
        string_manager.vibro_main = parser.intval('A');
        if(string_manager.vibro_main!=1)
        {
            hal.set_pwm_duty(pin_t(VIBRO1_PIN), 0);
             hal.set_pwm_duty(pin_t(VIBRO2_PIN), 0);
        }
  } 

   if (parser.seen('B')) 
  {
        string_manager.vibro_loop_high = parser.intval('B');
  } 

     
   if (parser.seen('C')) 
  {
        string_manager.vibro_loop_ampl = parser.intval('C');
  } 


  if (parser.seen('D')) 
  {
       string_manager.vibro_loop_ampl2 = parser.intval('D');
        //string_manager.power_m_vibro = parser.floatval('D');

  } 

  #endif

}
