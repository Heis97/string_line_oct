#pragma once


#include "../inc/MarlinConfig.h"
//#include "../libs/Adafruit_MAX31865_sw.h"
#include "../feature/twibus.h"
#include "../libs/MAX6675.h"
#include "../libs/MAX31865.h"
//#include "../libs/AS5048A.h"
//#include "../libs/AS5600.h" 
#include "../libs/Adafruit_PCF8574.h"
#include "../libs/MCP4725.h"
#include "../libs/ADS1X15.h"
//#include "../libs/HX711.h"
//#include "../libs/SparkFun_I2C_Mux_Arduino_Library.h"

#include "../libs/ethernet/Ethernet3.h"

#include "stepper.h"
#include "planner.h"
#include "endstops.h"

#include "../gcode/gcode.h"
#include "../gcode/parser.h"
#include "../gcode/queue.h"


#define PFLED_NET 1
#define PFLED_TURBO 2
#define PFLED_PNEVMO 9
#define PFLED_RECUP 5
#define PFLED_HEAT 6

#define PFLED_STRING0 0
#define PFLED_STRING1 3
#define PFLED_STRING2 8
#define PFLED_STRING3 4
#define PFLED_STRING4 7

#define VIBRO2_PIN  PA3// PA3



#define LED_MC1_PIN FAN0_PIN
#define LED_MC2_PIN FAN1_PIN

#define FAN_PEREPH_PIN FAN4_PIN
#define FAN_BOX_PIN FAN5_PIN

#define TURBO_PIN HEATER_3_PIN

#ifndef MAKET
#define PRESS_PIN HEATER_3_PIN
#endif

#ifndef MAKET
#define LED_POUND FAN2_PIN
#define VIBRO1_PIN  FAN3_PIN// PA3
#else
#define LED_POUND FAN3_PIN
#define VIBRO1_PIN  FAN2_PIN// PA3

#endif

class StringPeriphery {

public:

MCP4725 mcp4725_turbo = MCP4725(0x60);

//MCP4725 mcp4725_hv_i = MCP4725(0x60);
MCP4725 mcp4725_hv_v = MCP4725(0x61);
MCP4725 mcp4725_press = MCP4725(0x60);
ADS1015 analog_inp = ADS1015(0x48);
Adafruit_PCF8574 pcf_led1;
Adafruit_PCF8574 pcf_led2;

//QWIICMUX mux_iic = QWIICMUX();
//HX711 tensosensor = HX711();
//Adafruit_MAX31865_sw max_test1 = Adafruit_MAX31865_sw(TEMP_0_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
//AS5048A a5048_lin = AS5048A(ENC_LIN_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
//AS5600 as5600_lin = AS5600();


#define TENSOMETR_NUM 5
#define MED_FILTR_LEN 3



MAX6675 max6675_temp_cam_ext = MAX6675(TEMP_0_CS_PIN ,ARD_MOSI_PIN,ARD_MISO_PIN,ARD_SCK_PIN);
MAX6675 max6675_temp_cam_intern_1 = MAX6675(TEMP_1_CS_PIN ,ARD_MOSI_PIN,ARD_MISO_PIN,ARD_SCK_PIN);
MAX6675 max6675_temp_cam_intern_2 = MAX6675(TEMP_2_CS_PIN ,ARD_MOSI_PIN,ARD_MISO_PIN,ARD_SCK_PIN);

MAX31865 max_test1 = MAX31865(TEMP_0_MAX31865 ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);


void init();
void idle();

//-----------------M576
float get_vel();

int16_t get_pos();
void set_vel_lin(float v);
void set_vel_gateway(float v);
void set_vel_powder(float v);
void set_turbine(float v);

//-----------------M577
float get_v_hv();
float get_a_hv();
void set_hv_v(uint16_t v);
void set_hv_i(uint16_t v);

//-----------------M578
float get_v_press();
void set_press(uint16_t v);
void motors_free(uint8_t v);



void tare_tens();
void tare_tens_handler();
void tare_tens_handler_one(uint8_t v);
void tare_tens_pull(uint8_t v);
void tare_tens_release(uint8_t v);

//-----------------M579

float get_temp_cam_ext();
float get_temp_cam_intern1();
float get_temp_cam_intern2();

void set_reley_1(int v);
void set_reley_2(int v);
void set_reley_heater(int ind,int v);

void set_reley_HV(int v);
void set_reley_press(int v);

void set_24v_out(int v);
void set_24v_reset(int v);


void set_heater_2(int v);
void set_heater_3(int v);

void set_heaters_enable(int v);
void set_heaters_ind(int v);
void set_heaters_temp(float v);

void manage_heat();
void manage_heat_duty(float temp);
int manage_heat_duty_single(int ind, float temp, float kp);
void heat_pwm_control();
void heat_pwm_control_single(int ind, int counter,int duty);

void report_state();
void set_reporting(bool state);

//M580----------------------------------- 

void set_current_lenght_string(float dist);
void set_dest_lenght_string(float dist);
void move_pos_string(int val);
void move_pos_string_handler();

void manage_motion();

//---------------------
int manage_axis_vibro(AxisEnum Axis, int vibr, int k, int k_m,int dir,int vibr_a);
int manage_axis_vibro_simple(AxisEnum Axis,int dir,  int vibr, int k, int k_m);
void manage_axis(AxisEnum Axis, int dir);
int get_request_i2c(int adr, char rec_i2c[]);
bool parse_data_ts(char _data[], float* force, long* string_len,float force_k, uint8_t start_ind);
uint8_t crc8(const void* data, size_t length, uint8_t crc);
uint16_t crc16(const uint8_t *data, size_t length);
bool parse_data_ts_bin(char _data[], float* force, long* string_len,float force_k, uint8_t start_ind);
int* divide_data_parse(char _data[],int inds[]);
//M585---------------------------

void set_turbo(uint16_t v);


//string----------------------------------
void comp_speeds_string();
double comp_one_tension(double koef_tens, double koef_v_tens, float force_cur, float force_dest,int en);


//micros-------------M585----------------------------

void motors_free_bunker(uint8_t v);

void move_depth_d(float dist);
void move_betw_string_d(float dist);
void move_to_pos_d(int num);
void apply_limits_d(float _cam_coord_in,float _mir_coord_in);
void home_d(uint8_t v);

void move_depth_e(float dist);
void move_betw_string_e(float dist);
void move_to_pos_e(int num);
void apply_limits_e(float _cam_coord_in,float _mir_coord_in);
void home_e(uint8_t v);

void move_one_axis(AxisEnum ax, float dist);
void move_two_axis(AxisEnum ax1, float dist1,AxisEnum ax2, float dist2);

String state_cur();
String state_cur_sup();

void string_ethernet_begin_3();
void string_ethernet_loop_3();

void test_ethernet_loop();

void string_tcp_ethernet_begin();
void string_tcp_ethernet_loop_2();


void string_spi_begin();
void test_spi_loop_part1(uint8_t num);
void string_spi_loop();
//-----------------------------------

void refresh_val(float val, int row,float arr[TENSOMETR_NUM][MED_FILTR_LEN]);
float med_filtr(int row,float arr[TENSOMETR_NUM][MED_FILTR_LEN]);
void refresh_val(long val, int row,long arr[TENSOMETR_NUM][MED_FILTR_LEN]);
long med_filtr(int row,long arr[TENSOMETR_NUM][MED_FILTR_LEN]);


void set_vel_strings(float vel);
void reset_vel_strings();



void set_pfled(uint8_t numled,bool val);
void handler_pfled();
void refresh_pfled();
//__________________________________


int vibro_main = 0;


long cur_line_num = 0;
#ifdef MAKET
float kp_1 = 40;//
float kp_2 = 40;//
#else
float kp_1 = 140;//
float kp_2 = 140;//
#endif
int cycle_time = 10000;
int period_manage_mcs = 1000;

int temp_duty_manage = 1;

float temp_koef_k = 1;
float temp_koef_b = 0;

volatile int gateway_move = 0;
volatile int recuperator_move = 0;
volatile int feed_pound_move = 0;

int motors_free_state = 0;
int tare_tens_state = 0;

int sec_remain = 0;

double r_tens = 39;//d
volatile double k_enc_abs = 1.0d;

float cur_speed_sec_med = 0.0f;



volatile float gateway_def_vel = 3.0f;
volatile float feed_pound_def_vel = 10.0f;
volatile float recuperator_def_vel = 12.0f;

volatile float karet_def_vel = 4.0f;

volatile float taring_pull_vel = 1.0f;
volatile float taring_release_vel = 1.0f;

volatile float vibro_vel = 10.0f;//9
volatile float vibro_vel_valve = 80.0f;
volatile float vibro_vel_prim = 80.0f;
float vibro_ampl_valve = 40.0f;
float vibro_ampl2_valve = 1.0f;

int string_vibro = 0;
int karet_move = 0;

volatile int string_move = 0;
volatile int string_move_second[5] = {0,0,0,0,0};

volatile int string_state_second[5] = {0,0,0,0,0};

int vibro_phase = 0;
int vibro_time = 400.0;
int relax_time = 1600.0;
float s_y_def = 200;
float s_y_vibr = 1600;

float dist_m = 0.05;
int steps_m = 1000;
int buff_m = 11;



int vibro_loop_high = 2000;
int vibro_loop_ampl = 250;
int vibro_loop_ampl2 = 250;
int vibro_loop_ampl_cur = 0;



int k_m[8] =    {1,1,1,1,1,1,1,1};
int vibr[8] =   {0,0,0,0,0,0,0,0};
int vibr_a[8] = {1,1,1,1,1,1,1,1};
int k[8] =      {1,1,1,1,1,1,1,1};
int dir[8] =    {1,1,1,1,1,1,1,1};


uint16_t turbo_val_cur = 0;

//byte karet_ind = 5;
//byte motor_com_ind = 6;

byte karet_axis = (byte)X_AXIS;
byte motor_com_axis =(byte) Y_AXIS;
byte motors_tens[5] = {(byte)Z_AXIS,(byte)I_AXIS,(byte)J_AXIS,(byte)K_AXIS,(byte)U_AXIS};

byte feed_pound_axis = (byte)X_AXIS;
byte gateway_axis = (byte)Y_AXIS;
byte recuperator_axis = (byte)Z_AXIS;

byte vibro_axis = (byte)E_AXIS;

EthernetClient client;

double koef_v_tens_one = 0.00001;
double koef_tens[5] =      {1,1,1,1,1};
double koef_v_tens[5] =      {koef_v_tens_one,koef_v_tens_one,koef_v_tens_one,koef_v_tens_one,koef_v_tens_one};

int string_direction = 1;
float koef_gauss[5] =  {1,1,1,1,1};  

float cur_speed_enc[5] =         {1,1,1,1,1};    
float cur_speed_tens[5] =         {1,1,1,1,1};     float cur_speed_tens_com = 1;
float orig_speed_tens[5] =         {1,1,1,1,1};     float orig_speed_tens_com = 1;

float steps_mm[5] =         {90,90,90,90,90};     float steps_mm_com = 90;
float default_steps_mm[5] = {90,90,90,90,90};     float default_steps_mm_com = 90;

float speed_koef = 1;
float force_cur[5] = {0,0,0,0,0};
float force_dest[5] = {-30,-30,-30,-30,-30};
float force_off[5] = {0,0,0,0,0};
float force_k[5] ={1,1,1,1,1};

long string_lenght_cur[5] = {0,0,0,0,0};
long string_lenght_int[5] = {0,0,0,0,0};
long string_lenght_off[5] = {0,0,0,0,0};
long string_lenght_dest[5] = {0,0,0,0,0};

bool taring_process[TENSOMETR_NUM] = {false,false,false,false,false};
bool taring_process_all = false;


#ifndef PRIMARY_PLATE
//----------D micro----------------------


void set_led_micro_d(uint8_t v);
void set_led_micro_e(uint8_t v);

volatile float microsc_vel = 5.1f;


AxisEnum mirror_axis_d = J_AXIS;
AxisEnum camera_axis_d = I_AXIS;
float mirror_coord_d = 0.0;
float camera_coord_d = 0.0;

float mirror_h_off_d = 0.0;
float camera_h_off_d = 0.0;
float camera_v_off_d = 0.0;
int homed_d = 0;


float mirror_cur_d = 0;
float camera_cur_d = 0;
float offset_mirror_d = 3;//10 maket // 3
//float pos_mirror_d[3] = {1,2,3};
//float pos_camera_d[3] = {1,2,3};

float pos_mirror_d[5] = {2,7,15,22,30};
float pos_camera_d[5] = {-5,2,8,16,23};

float limit_camera_d = 50;
float limit_mirror_d = 50;

int led_micr_d = 0;

//----------E micro----------------------
AxisEnum mirror_axis_e = U_AXIS;
AxisEnum camera_axis_e = K_AXIS;
float mirror_coord_e = 0.0;
float camera_coord_e = 0.0;

float mirror_h_off_e = 0.0;
float camera_h_off_e = 0.0;
float camera_v_off_e = 0.0;
int  homed_e = 0;


float mirror_cur_e = 0;
float camera_cur_e = 0;
float offset_mirror_e = 3;//10
//float pos_mirror_e[3] = {1,2,3};
//float pos_camera_e[3] = {1,2,3};

float pos_mirror_e[5] = {2,7,15,22,30};
float pos_camera_e[5] = {-5,2,8,16,23};

float limit_camera_e = 50;
float limit_mirror_e = 50;

int led_micr_e = 0;
#endif

private:

unsigned long time_measure,time_measure_enc,time_measure_temp;
float temp_val_int1 = 0;
float temp_val_int2 = 0;
float temp_val_ext = 0;
int reley_1,reley_2,reley_HV,reley_24_out,reley_press, turbo, string_cur_pos, string_last_pos,moves_planned;//

float pressure = 0;//
int pressure_dest = 0;//
float HV = 0;//

float temp_dest = 0;
int ind_sensor = 0;
int heater_en = 0;
float temp_hyst = 15;
bool reporting = true;

bool heating_1,heating_2;
int duty_1 = 0;
int duty_2 = 0;

int duty_time_1 = 0 ; 
int duty_time_2 = 0 ; 


int duty_counter = 0;


};
extern StringPeriphery string_manager;
