

#define AXIS_NUM 8
#include <Arduino.h>
#include "../inc/MarlinConfig.h"
#include "../module/settings.h"
#include "../module/planner.h"





class StepDirDriverPos {

  public:
    /*StepDirDriverPoz(byte pinStep, byte pinDir, byte pinEn); // конструктор
    void  control();  // управление, метод должен вызываться регулярно с максимальной частотой коммутации фаз
	void  gotopoz(long int koord);
	void  step(long int steps);  // инициирует поворот двигателя на заданное число шагов
    void  setMode(byte stepMode, boolean fixStop);  // задает режимы коммутации фаз и остановки
    void  setPoz(long int poz); 
	void  setDivider(long int divider);  // установка делителя частоты для коммутации фаз
   long  int readSteps();  // чтение оставшихся шагов
	long int readPoz();  // чтение координаты*/

      StepDirDriverPos(int* pinStep, int* pinDir, int* pinEn, int* pinStop); // конструктор
      void  control();  // управление, метод должен вызываться регулярно с максимальной частотой коммутации фаз.
      void  control(byte num); 
      void  home_axis(byte num);
      void  home_handler(byte num);
      void  home_handler();
      void  gotopos(long int koord, byte num);
      void  gotopos(float dist, byte num);
      void  step(long int steps, byte num);  // инициирует поворот двигателя на заданное число шагов
      void  step(float dist, byte num);  // инициирует поворот двигателя на заданное число шагов
      void  setPos(long int pos, byte num); 
      void  setPos(float dist, byte num); 
      void  setDivider(long int divider, byte num);  // установка делителя частоты для коммутации фаз
      void  setVel(float vel, byte num);
      void  set_motor_dir(int dir, byte num);
      volatile long int* readSteps();  // чтение оставшихся шагов
      volatile long int* readPos();  // чтение координаты
      volatile bool* readHoming();
      volatile bool readHoming_one(byte ax);
      long int dist_to_steps(float dist, byte num);
      byte readEnd(byte num);

      void idle();

      void wake_up(byte num);
      void wake_up_all();
      void sleep(byte num);
      void sleep_all();
      void sleep_string(byte motors_tens[5]);
//DEFAULT_AXIS_STEPS_PER_UNIT

#ifndef PPRIMARY_PLATE
      volatile int _vibro[AXIS_NUM]{ }; 
      volatile int vibro_ampl[AXIS_NUM] { };
      volatile int vibro_counter[AXIS_NUM] {};
      volatile int cur_dir[AXIS_NUM] {};
      #endif
    private:
      volatile long int _steps[AXIS_NUM]{};// оставшееся число шагов 
      volatile long int _pos[AXIS_NUM]{}; 
        
     // boolean _fixStop[AXIS_NUM];  // признак фиксации положения при остановке
      volatile  long  int  _divider[AXIS_NUM]{};  // делитель частоты для коммутации фаз
      volatile long  int  _dividerCount[AXIS_NUM]{};  // счетчик делителя частоты для коммутации фаз
      int _pinStep[AXIS_NUM]{};
      int  _pinDir[AXIS_NUM]{};
      int  _pinEn [AXIS_NUM]{};
      int  _pinStop [AXIS_NUM]{};

      
      //long int koord[AXIS_NUM]{};     
      


      
      

} ;
extern StepDirDriverPos motors;