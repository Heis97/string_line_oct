#include <Wire.h> 
#include "HX711.h"
#define I2C_REC_LEN 25
HX711 scale;
uint8_t dataPin = 3;
uint8_t clockPin = 2;
 char mes[I2C_REC_LEN];
long string_lenght = 0;
int string_last_pos = 0;
int f;
#include <iarduino_I2C_Software.h> 
#include "AS5600.h"
SoftTwoWire sWire(7,8);
AS5600 encoder = AS5600(&sWire);


unsigned long time_measure;
unsigned long cur_time;
    unsigned long dt ;
    unsigned long dt_glob;
void setup(){                            
     Wire.begin(81);  //80 - enc1, 81 - enc2, 82 - enc3, 83 - enc4            
     Wire.onRequest(request_i2c);        
     Wire.onReceive(decod_main_i2c);                
     sWire.begin();                            
     Serial.begin(250000);
     //Serial.println("1");

     scale.begin(dataPin, clockPin);
}                                    
                                     
void loop(){          
    cur_time = millis();
    dt = (cur_time - time_measure);
    
    if(dt>10)//every 10ms
    { 
      get_data_enc();      
      dt_glob+=dt;      
       if(dt_glob>110)//every 110ms
       {
         get_data_tens();
         time_measure = cur_time;
       }
    }
} 

char data_lenght[12];
void get_data_tens()
{
     String mes_str ="";
     char data_tens[12];
     long tens = (long)scale.read();
     ltoa(tens,data_tens, 10);
     mes_str = "kbd"+String(data_tens)+"s"+String(data_lenght)+"c";
     while(mes_str.length()<I2C_REC_LEN) mes_str+="k";
     mes_str.toCharArray(mes,sizeof(mes));
     Serial.println(mes);
}


void get_data_enc()
{
     int string_cur_pos = encoder.rawAngle();
     int d_pos = string_last_pos - string_cur_pos;
     if(d_pos>2047) d_pos = (4096 - string_last_pos )+ string_cur_pos;
     else if(d_pos<(-2047)) d_pos = (4096 + string_last_pos )- string_cur_pos;
     string_lenght = string_lenght + d_pos;
     string_last_pos = string_cur_pos;
     ltoa(string_lenght,data_lenght, 10); 
}
void request_i2c()
{
   Wire.write(mes);  
}


void decod_main_i2c()
{
  int temperature_control = 0;
  int resp = 0;
  if (Wire.available())
  {
    int byte1 = Wire.read() - 48;
    Serial.println(byte1);
    if (byte1 == 50)
    {
      
      int inserial[8];
      for (int i = 0; i < 6; i++)
      {
        inserial[i] =Wire.read() - 48;
        if (inserial[i] == -49 )
        {
          resp = 9;
          Serial.println(inserial[i]);
          //break;
        }
      }

      if (resp != 9)
      {
        
        byte _adr = inserial[6] * 10 + inserial[7];
        int _var = inserial[4] * 10 + inserial[5];
        int _val = inserial[0] * 1000 + inserial[1] * 100 + inserial[2] * 10 + inserial[3];
        switch (_var) {


          case 30: temperature_control = _val; break;
        }
        Serial.print("var: ");
        Serial.print(_var);
        Serial.print(": ");
        Serial.println(_val);
        
        while(Wire.available())
        {
          Wire.read();
        }
      }
    }
  }
}
