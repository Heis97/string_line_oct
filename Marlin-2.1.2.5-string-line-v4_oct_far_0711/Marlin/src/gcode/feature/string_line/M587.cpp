//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M587() {
    #ifdef PRIMARY_PLATE
    if (parser.seen('I') && parser.seen('S'))  motors.step(parser.floatval('S'),parser.byteval('I')); 
    if (parser.seen('I') && parser.seen('P'))  motors.gotopos(parser.floatval('P'),parser.byteval('I')); 
    if (parser.seen('I') && parser.seen('M'))  motors.setVel(parser.floatval('M'),parser.byteval('I')); 
    if (parser.seen('I') && parser.seen('H'))  motors.home_axis(parser.byteval('I')); 
    //if (parser.seen('Q') && parser.seen('E'))  if (string_manager.client) { string_manager.client.stop();}
     if (parser.seen('I') && parser.seen('D'))  motors.set_motor_dir(parser.intval('D'),parser.byteval('I')); 

    if (parser.seen('F'))  { motors.sleep_all();}
    if (parser.seen('W'))  { motors.wake_up_all();}


    if (parser.seen('U'))  { motors.sleep_string(string_manager.motors_tens);}

    #endif
}
