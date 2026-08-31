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
#define K_SET 0.63
int motor_dir[AXIS_NUM] {-1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  };
float steps_pr_mm_orig[AXIS_NUM] { 800, 80*K_SET, 120*K_SET,120*K_SET, 120*K_SET, 120*K_SET, 60*K_SET, 100 }; //speed x1.5 string//speed not right ~20%
float steps_pr_mm[AXIS_NUM] { 800,80*K_SET, 120*K_SET,120*K_SET, 120*K_SET, 120*K_SET, 60*K_SET, 100}; //speed x1.5 string//speed not right ~20%
float steps_pr_mm_k[AXIS_NUM] { 1, 1, 1, 1, 1, 1, 1, 1};
#else
int motor_dir[AXIS_NUM] {1,  1,  -1,  -1,  1,  -1,  -1,  1  };
float  steps_pr_mm_orig[AXIS_NUM] {  800, 800, 800,400, 400, 1600, 800, 50 };
float  steps_pr_mm[AXIS_NUM] {  800, 800, 800,400, 400, 1600, 800, 50 };
float steps_pr_mm_k[AXIS_NUM] { 1, 1, 1, 1, 1, 1, 1, 1};
#endif

#else

#ifdef PRIMARY_PLATE
//dev 1    {-1,  -1,  -1,  1,  1,  1,  1,  1  };
//dev 2 
//

//dev 10    {-1,  -1,  1,  1,  1,  1,  1,  1  };
int motor_dir[AXIS_NUM] {-1,  -1,  -1,  -1,  -1,  -1,  -1,  1 };
//int steps_pr_mm[AXIS_NUM] { 800, 160, 240,240, 240, 240, 120, 100  };

float steps_pr_mm_orig[AXIS_NUM] { 800, 80, 94,94, 94, 94, 47, 100  };
float steps_pr_mm[AXIS_NUM] { 800, 80, 94,94, 94, 94, 47, 100  };
float steps_pr_mm_k[AXIS_NUM] { 1, 1, 1, 1, 1, 1, 1, 1};
#else
int motor_dir[AXIS_NUM] {1,  1,  1,  -1,  -1,  -1,  -1,  1  };
float steps_pr_mm_orig[AXIS_NUM] { 800, 800, 800,400, 400, 400, 200, 400 };
float steps_pr_mm[AXIS_NUM] { 800, 800, 800,400, 400, 400, 200, 400 };
float steps_pr_mm_k[AXIS_NUM] { 1, 1, 1, 1, 1, 1, 1, 1};
#endif

#endif

volatile bool do_step[AXIS_NUM]{false,false,false,false,false,false,false,false}; 
volatile bool _homing_need[AXIS_NUM]{false,false,false,false,false,false,false,false};     
volatile bool _homed[AXIS_NUM]{false,false,false,false,false,false,false,false};     

//#ifndef PRIMARY_PLATE
volatile int _vibro[AXIS_NUM]{0,  0,  0,  1,  1,  1,  1,  1 }; 
volatile int vibro_ampl[AXIS_NUM] {15,  15,  15,  10,  10,  10,  10,  10  };
volatile int vibro_counter[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
volatile int cur_dir[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
//#endif
int count_handl = 0;
#define DIV_VEL_ZERO 100000
StepDirDriverPos motors(step_pins, dir_pins, en_pins, stop_pins);

//#define DEBUG_STEP_DIR
#define DEBUG_STEP_DIR_TARGET 5
#define COUNT_HAND_END 300
//---------------------------- конструктор -----------------------------------
StepDirDriverPos::StepDirDriverPos (int* pinStep, int* pinDir, int* pinEn, int* pinStop) {
 // Serial.println("StepDirDriverPos::StepDirDriverPos");
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
    _divider[i] = DIV_VEL_ZERO;
    _divider_sub[i] = 0;
    _dividerCount[i] = 0;   
    _pos[i] = 0;
    
    do_step[i] = false;
    _homing_need[i] = false;

    #ifndef PRIMARY_PLATE 
    setVelDest(2.1,i);
    setAcs(0.5f,i);
    if(i>=3)
    {
       setVelDest(2.5,i);
       setAcs(0.6f,i);
    }
    #else
    setVelDest(2.1,i);
    setAcs(10.0f,i);
    #endif
   
    _vel_prev[i] = 0;
      vibro_ampl[i] = 15;
    
  }
vibro_ampl[E0_AXIS] = 30;
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
    else 
    { 
      _dividerCount[num]= 0;
      if(_dividerCount_sub[num] > _divider_sub[num])
      {
        _dividerCount[num]= 1;
      };
      _dividerCount_sub[num]++;
      if(_dividerCount_sub[num]==100)
      {
        _dividerCount_sub[num] = 0;
      }
    };
  
 
	  if (_steps[num] > 0) 
	  { _steps[num]--; _pos[num]++; } // вращение против часовой стрелки
	  else
	  { _steps[num]++; _pos[num]--; }// вращение по часовой стрелке           
 
  

  if ( _steps[num] != 0 ) {
    //Serial.println("do_step true");
    WRITE(_pinStep[num], HIGH); 
    do_step[num] = true;     


    //#ifndef PRIMARY_PLATE
    if(_vibro[num]==1)
    {
       _steps[num]  =200;
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

    //#endif

  }           
}

void  StepDirDriverPos::control() {
  //for (byte i=AXIS_NUM-1; i>0;i--){ control(i); }
  control(7);
  control(6);
  control(5);
  control(4);
  control(3);
  control(2);
  control(1);
  control(0);
}
//------------------------------- запуск вращения
// инициирует поворот двигателя на заданное число шагов
void  StepDirDriverPos::step(long int steps, byte num) { 
  

  if(steps==0 ) {_steps[num]= 0; 
    #ifndef PRIMARY_PLATE
    _vel[num] = 0;_vel_prev[num] = 0;
    #endif
     return;}
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

void StepDirDriverPos::set_cur_k(float k, byte num)  
{
  if(k<0) k = 0.00001f;
  steps_pr_mm_k[num] = k;
  steps_pr_mm[num] =  steps_pr_mm_k[num]*steps_pr_mm_orig[num];
}


void StepDirDriverPos::setVel(volatile float vel, byte num)  
{
  volatile float vel_steps =(volatile float)(vel*steps_pr_mm[num]);

  if(vel<0.0001) _vel_dest[num] = 0.0001;
  else           _vel_dest[num] = vel_steps;

  #ifdef DEBUG_STEP_DIR
  if(num==DEBUG_STEP_DIR_TARGET)
  {
    Serial.print("set vel ");
    Serial.println((float)_vel_dest[num]);
  }

  #endif
  
  setVelIntern(_vel_dest[num],num);
}

void StepDirDriverPos::setDiv(volatile float div, byte num)  
{
  double integerPart;
  double fractionalPart;

  fractionalPart = modf(double(div), &integerPart);

  _divider[num] = (long)integerPart;
  _divider_sub[num] = (int)(fractionalPart*100);
}

void  StepDirDriverPos::setVelIntern(volatile float vel, byte num)
{
  volatile float _vel_ch = vel;
  if(_vel_ch==0) _vel_ch = 0.01f;
  double div = (100000.0f/vel);//FREQ_MOTORS
   #ifdef DEBUG_STEP_DIR
  if(num==DEBUG_STEP_DIR_TARGET && count_handl == COUNT_HAND_END-1)
  {
    Serial.print("vel ");
    Serial.print((float)vel);
    Serial.print("div ");
    Serial.print(div);
    Serial.print("_vel_dest ");
    Serial.println((float)_vel_dest[DEBUG_STEP_DIR_TARGET]);
  }
  #endif
  
  if(div<2) div = 2;
  _vel[num] = _vel_ch;
  setDiv(div,num);
  //_divider[num]  = (volatile long)div; 
};

void  StepDirDriverPos::setAcs(float acs, byte num)
{
  _acs[num] =  acs*steps_pr_mm[num]/1000.0f;
};

void StepDirDriverPos::setVelDest(volatile float vel, byte num)
{
  _time_ch_vel_prev[num] = micros();
  _vel_prev[num] = _vel[num];
  _vel_dest[num] =  vel*steps_pr_mm[num];
  #ifdef DEBUG_STEP_DIR  
  Serial.print("vel ");
  Serial.print((float)_vel[num]);
  Serial.print(";vel_dest ");
  Serial.println((float)_vel_dest[num]);
  #endif

};

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
volatile long  StepDirDriverPos::readPosOne(byte num)  {
  //long int poz;
  //poz = _poz;
  return _pos[num];
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

float StepDirDriverPos::readVelDest(byte num)
{
  return _vel_dest[num];
};

float StepDirDriverPos::readVel(byte num)
{
  return _vel[num];
};
long int StepDirDriverPos::dist_to_steps(float dist, byte num)
{
   return (long int)(dist*steps_pr_mm[num]);
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

void  StepDirDriverPos::vel_handler(byte _num)
{
    if(_steps[_num]== 0) {
       _vel[_num]= 0; 
       return;
      }
    unsigned long  _dt_time_ch_vel = _time_ch_vel-_time_ch_vel_prev[_num];

    if(_vel[_num]>=_vel_dest[_num])
    {
      
      _vel[_num] = _vel_dest[_num];
      //return;
    }
    else
    {
     
    
      _vel[_num]= _vel_prev[_num] +_acs[_num]*((volatile float)_dt_time_ch_vel)/1000;


      if(_vel[_num]>=_vel_dest[_num])
      {
        _vel[_num]==_vel_dest[_num];
      }

      #ifdef DEBUG_STEP_DIR

    if(_num==DEBUG_STEP_DIR_TARGET)// && count_handl==COUNT_HAND_END-1)
    {
      /*Serial.print( _time_ch_vel_prev[_num]);
      Serial.print(" ");
      Serial.print( _time_ch_vel);
      Serial.print(" ");
      Serial.print( _dt_time_ch_vel);
      Serial.print(" ");
      Serial.print(_acs[_num]*((float)_dt_time_ch_vel)/1000);
      Serial.print(" ");
      Serial.print(_vel_dest[_num]);
      Serial.print(" ");
      Serial.print(_vel_prev[_num]);
      Serial.print(" ");
      Serial.print(_vel[_num]);
      Serial.print(" ");
      Serial.print(_steps[_num]);
      Serial.print(" ");
      Serial.println(_divider[_num]);*/
    }

    #endif

      
      
    }
    setVelIntern(_vel[_num],_num);
    
}


void  StepDirDriverPos::vel_handler()
{
  

_time_ch_vel = micros();
 // for (byte i=0; i<AXIS_NUM;i++){  vel_handler(i); }  

    vel_handler(0);
    vel_handler(1);
    vel_handler(2);
    vel_handler(3);
    vel_handler(4);
    vel_handler(5);
    vel_handler(6);
    vel_handler(7);

  #ifdef DEBUG_STEP_DIR

  if(count_handl<COUNT_HAND_END)
  {    
    count_handl++;
  }
  else
  {
    count_handl = 0;
    /*Serial.print(_steps[2]);
    Serial.print(" ");
    Serial.print( _time_ch_vel);
    Serial.print(" ");
    Serial.println(_divider[2]);*/
  }

  #endif
}
int counter_idle = 0;
void StepDirDriverPos::idle()
{
 // #ifndef PRIMARY_PLATE
  counter_idle++;
  if(counter_idle>100)
  {
    vel_handler();
    counter_idle=0;
  }
 //#endif
  home_handler();
}