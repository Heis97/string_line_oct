#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/temperature.h"
#include "../../../module/string_periphery.h"


//#include "../../../module/motion.h"


void GcodeSuite::M579() {
  #ifdef PRIMARY_PLATE
  if (parser.seen('S') && parser.seen('I')) 
  {
    switch (parser.intval('I'))
    {
    case 0: string_manager.set_reley_1(parser.intval('S')); break;
    case 1: string_manager.set_reley_2(parser.intval('S')); break;
    case 2: string_manager.set_reley_HV(parser.intval('S')); break;
    case 3: string_manager.set_reley_press(parser.intval('S')); break;    
    case 4: string_manager.set_heater_2(parser.intval('S')); break; 
    case 5: string_manager.set_heater_3(parser.intval('S')); break; 
    case 6: string_manager.set_24v_out(parser.intval('S')); break; 
    case 7: string_manager.set_24v_reset(parser.intval('S')); break; 
    default:
      break;
    }
  } 
  if (parser.seen('G')) 
  {
    if(parser.intval('G')==0)
    {
      string_manager.get_temp_cam_ext(); 
    }
    else if(parser.intval('G')==1)
    {
      string_manager.get_temp_cam_intern1();
    }
    else if(parser.intval('G')==2)
    {
      string_manager.get_temp_cam_intern2();
    }
  }
  
  if (parser.seen('T')) 
  {
      string_manager.set_heaters_temp(parser.floatval('T'));

      
  }

  if (parser.seen('E')) 
  {
    string_manager.set_heaters_enable(parser.floatval('E'));
    
  }

  if (parser.seen('A')) 
  {
    string_manager.temp_duty_manage = parser.intval('A');
    
  }

  if (parser.seen('B')) 
  {
    string_manager.temp_koef_b = parser.floatval('B');
    
  }

  if (parser.seen('C')) 
  {
    string_manager.temp_koef_k = parser.floatval('C');
    
  }

  if (parser.seen('C') && parser.seen('S')&& parser.seen('E')) 
  {
    M303();
  }
  if (parser.seen('P') && parser.seen('I')&& parser.seen('D'))
  {
    M301();
  }
  if (parser.seen('D')) 
  {
    string_manager.kp_1 = parser.floatval('D');
    string_manager.kp_2 = parser.floatval('D');
  }
  if (parser.seen('N')) 
  {
    string_manager.set_heaters_ind(parser.intval('N'));
  }

  if (parser.seen('K') ) 
  {
    string_manager.kp_1 = parser.floatval('K');
  }
  if (parser.seen('P') ) 
  {
    string_manager.kp_2 = parser.floatval('P');
  }

    
  if (parser.seen('L')) 
  {
      float time_s = parser.floatval('L');
   //   string_manager.cycle_time = (int)((1000000*time_s) / (float)string_manager.period_manage_mcs);
  }
  #endif
}
