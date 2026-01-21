/*
StepDirDriverPos.h - библиотека управления STEP/DIR драйвером шагового двигателя

*/

#include "StepDirDriverPos.h"



int step_pins[AXIS_NUM] {X_STEP_PIN,  Y_STEP_PIN,  Z_STEP_PIN,  I_STEP_PIN,  J_STEP_PIN,  K_STEP_PIN,  U_STEP_PIN,  E0_STEP_PIN  };
int dir_pins [AXIS_NUM] {X_DIR_PIN,   Y_DIR_PIN,   Z_DIR_PIN,   I_DIR_PIN,   J_DIR_PIN,   K_DIR_PIN,   U_DIR_PIN,   E0_DIR_PIN   };
int en_pins  [AXIS_NUM] {X_ENABLE_PIN,Y_ENABLE_PIN,Z_ENABLE_PIN,I_ENABLE_PIN,J_ENABLE_PIN,K_ENABLE_PIN,U_ENABLE_PIN,E0_ENABLE_PIN};
int stop_pins[AXIS_NUM] {X_DIAG_PIN,  Y_DIAG_PIN,  Z_DIAG_PIN,  I_DIAG_PIN,  J_DIAG_PIN,  K_DIAG_PIN,  U_DIAG_PIN,  E0_DIAG_PIN  };

#ifdef MAKET
#ifdef PRIMARY_PLATE
int motor_dir[AXIS_NUM] {-1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  };
int steps_per_mm[AXIS_NUM] { 800, 80, 120,120, 120, 120, 60, 100 };
#else
int motor_dir[AXIS_NUM] {1,  1,  1,  -1,  1,  -1,  -1,  1  };
int steps_per_mm[AXIS_NUM] { 100, 100, 100,400, 400, 1600, 800, 800 };

#endif

#else
#ifdef PRIMARY_PLATE
int motor_dir[AXIS_NUM] {-1,  1,  1,  1,  1,  1,  1,  1  };
//int steps_per_mm[AXIS_NUM] { 800, 160, 240,240, 240, 240, 120, 100  };

int steps_per_mm[AXIS_NUM] { 800, 80, 120,120, 120, 120, 60, 100  };
#else
int motor_dir[AXIS_NUM] {1,  1,  1,  -1,  -1,  -1,  -1,  1  };
int steps_per_mm[AXIS_NUM] { 100, 100, 100,400, 400, 400, 200, 100 };
#endif

#endif

volatile bool do_step[AXIS_NUM]{false,false,false,false,false,false,false,false}; 
volatile bool _homing_need[AXIS_NUM]{false,false,false,false,false,false,false,false};     
volatile bool _homed[AXIS_NUM]{false,false,false,false,false,false,false,false};     

#ifndef PPRIMARY_PLATE
volatile int _vibro[AXIS_NUM]{1,  1,  1,  1,  1,  1,  1,  1 }; 
int vibro_ampl[AXIS_NUM] {15,  15,  15,  10,  10,  10,  10,  10  };
int vibro_counter[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
int cur_dir[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
#endif
StepDirDriverPos motors(step_pins, dir_pins, en_pins, stop_pins);

//---------------------------- конструктор -----------------------------------
StepDirDriverPos::StepDirDriverPos (int* pinStep, int* pinDir, int* pinEn, int* pinStop) {
  for (byte i=0; i<AXIS_NUM;i++)
  {

    _pinStep[i] = pinStep[i];
    _pinDir [i] = pinDir[i];
    _pinEn  [i] = pinEn[i];
    _pinStop[i] = pinStop[i];
    SET_OUTPUT(_pinStep[i]); WRITE(_pinStep[i], LOW);
    SET_OUTPUT(_pinDir[i]); WRITE(_pinDir[i], LOW);
    SET_OUTPUT(_pinEn[i]); WRITE(_pinDir[i], LOW);
     
    SET_INPUT(_pinStop[i]);
    _steps[i] = 0;
    //_fixStop = false;
    _divider[i] = 100;
    _dividerCount[i] = 0;   
    _pos[i] = 0;
    
    do_step[i] = false;
    _homing_need[i] = false;

    setVel(1,i);
    #ifndef PPRIMARY_PLATE
      vibro_ampl[i] = 15;
    #endif

    
  }
}


//------------------------------- управление коммутацией фаз
// метод должен вызываться регулярно с максимальной частотой коммутации фаз
void  StepDirDriverPos::control(byte num) {
   //Serial.println(num);
   if(do_step[num]) { 
    WRITE(_pinStep[num], LOW);
     do_step[num] = false; 
     //Serial.println("do_step false");
    };
  // делитель частоты коммутации
  if ( _steps[num] == 0 ) return;
    //двигатель не остановлен
    _dividerCount[num]++;  
    if ( _dividerCount[num] < _divider[num] ) return;  
    _dividerCount[num]= 0;        
  
 
	  if (_steps[num] > 0) 
	  { _steps[num]--; _pos[num]++; } // вращение против часовой стрелки
	  else
	  { _steps[num]++; _pos[num]--; }// вращение по часовой стрелке           
 
  

  if ( _steps[num] != 0 ) {
    //Serial.println("do_step true");
    WRITE(_pinStep[num], HIGH); 
    do_step[num] = true;     


    #ifndef PRIMARY_PLATE
    if(_vibro[num]==1)
    {
       _steps[num]  =5;
      if(vibro_counter[num] < vibro_ampl[num])
      {
        vibro_counter[num]++;
      }
      else
      {
        vibro_counter[num] = 0;
        if(cur_dir[num]==0) 
        {
          WRITE(_pinDir[num],HIGH);
          cur_dir[num] = 1;
        }
        else
        {
          WRITE(_pinDir[num],LOW);
          cur_dir[num] = 0;
        }
      }
    }

    #endif

  }           
}

void  StepDirDriverPos::control() {
  //for (byte i=AXIS_NUM-1; i>=0;i--){ control(i); }
  control(6);
  control(5);
  control(4);
  control(3);
  control(2);
  control(1);
  control(0);
  //for (byte i=0; i<AXIS_NUM;i++) control(i);
}
//------------------------------- запуск вращения
// инициирует поворот двигателя на заданное число шагов
void  StepDirDriverPos::step(long int steps, byte num) { 
  

  if(steps==0 ) {_steps[num]= 0; return;}
  //Serial.println(steps);
  
  //WRITE(_pinEn[num], LOW);
  if (steps*motor_dir[num] < 0 )//*motor_dir[num]
   {
    WRITE(_pinDir[num], LOW);
    cur_dir[num] = 0;
    //Serial.println("dir low");
  }
  else 
  {
    WRITE(_pinDir[num], HIGH);
    cur_dir[num] = 1;
  //Serial.println("dir high");
  }
  _steps[num]= steps;
}

void  StepDirDriverPos::step(float dist, byte num) {

  long d = dist_to_steps(dist,num);
  step(d,num); 
}

void StepDirDriverPos::wake_up(byte num)
{
  WRITE(_pinEn[num], LOW);
}

void StepDirDriverPos::wake_up_all()
{
  for (byte i=0; i<AXIS_NUM;i++) wake_up(i);
}

void StepDirDriverPos::sleep(byte num)
{
  WRITE(_pinEn[num], HIGH);
}

void StepDirDriverPos::sleep_all()
{
  for (byte i=0; i<AXIS_NUM;i++) sleep(i);
}

void StepDirDriverPos::sleep_string(byte motors_tens[5])
{
  for (byte i=0; i<AXIS_NUM;i++) sleep(motors_tens[i]);
}

void  StepDirDriverPos::gotopos(long int koord, byte num) {
	  step(koord -_pos[num],num);
}


void  StepDirDriverPos::gotopos(float pos, byte num) {
	  gotopos(dist_to_steps(pos,num),num);
}
//------------------------------ режим коммутации фаз и остановки
//void  StepDirDriverPos::setMode(byte stepMode, boolean fixStop)  {  
//  _fixStop = fixStop;
//}

//------------------------------ установка делителя частоты для коммутации фаз
void StepDirDriverPos::setPos(long int pos, byte num)  {
  _pos[num] = pos;
} 

void StepDirDriverPos::setPos(float pos, byte num)  {
   setPos(dist_to_steps(pos,num),num);
} 


void StepDirDriverPos::setDivider(long int divider, byte num)  {
  _divider[num]  = divider; 
}


void StepDirDriverPos::setVel(float vel, byte num)  {

  float vel_steps = vel*(float)steps_per_mm[num];
  if (vel_steps==0) return;
  long div = (long)(100000.0f/vel_steps);//FREQ_MOTORS
   if(div<2) div = 2;

  _divider[num]  = (volatile long)div; 
}

void StepDirDriverPos::set_motor_dir(int dir, byte num)  {

  motor_dir[num] = dir;
}

//----------------------------- чтение оставшихся шагов 
volatile long int* StepDirDriverPos::readSteps()  {
  //long int stp;
  //stp = _steps;
  return _steps;
}

//----------------------------- чтение текущей координаты
volatile long int* StepDirDriverPos::readPos()  {
  //long int poz;
  //poz = _poz;
  return _pos;
}
volatile bool* StepDirDriverPos::readHoming()
{
  return _homing_need;
}

volatile bool StepDirDriverPos::readHoming_one(byte ax)
{
  return _homing_need[ax];
}

byte StepDirDriverPos::readEnd(byte num)
{
  return READ(_pinStop[num]);
}


long int StepDirDriverPos::dist_to_steps(float dist, byte num)
{
   return (long int)(dist*planner.settings.axis_steps_per_mm[num]);
}

void  StepDirDriverPos::home_axis(byte num)
{
  _homing_need[num] = true;
  step(-100000L,num);
}



void  StepDirDriverPos::home_handler(byte _num)
{
   if(!_homing_need[_num]) return;
    //Serial.println(READ(_pinStop[_num]));
    if(READ(_pinStop[_num])==0){
      _homing_need[_num] = false;
      _homed[_num] = true;
       step(0L,_num);
       setPos(0L,_num);
       step(0.1f,_num);       
    }
}

void  StepDirDriverPos::home_handler()
{
  for (byte i=0; i<AXIS_NUM;i++){  home_handler(i); }    
}

void StepDirDriverPos::idle()
{
  home_handler();
}