#include <Wire.h> 
#include "HX711.h"
#include <SPI.h> 
//#include "pins_arduino.h"
#define I2C_REC_LEN 25



#define I2C_ADDR1 80
#define I2C_ADDR2 81
#define I2C_ADDR3 82
#define I2C_ADDR4 83
HX711 scale;
uint8_t dataPin = 3;
uint8_t clockPin = 2;
 char mes[I2C_REC_LEN];

  char mes_s1[I2C_REC_LEN];
   char mes_s2[I2C_REC_LEN];
    char mes_s3[I2C_REC_LEN];
     char mes_s4[I2C_REC_LEN];
     
long string_lenght = 0;
int string_last_pos = 0;
int f;
#include <iarduino_I2C_Software.h> 
#include "AS5600.h"
SoftTwoWire sWire(7,8);
AS5600 encoder = AS5600(&sWire);

#define LEN_I2C 20

#define PIN_SPI_SS 10
#define PIN_SPI_MOSI  11
#define PIN_SPI_MISO  12
#define PIN_SPI_SCK  13
char buf[125];
char val[125];
volatile byte rcv_pos;
volatile byte xmt_pos = 0;
volatile boolean process_it;
volatile boolean bReceivingString = false;

unsigned long time_measure;
unsigned long time_measure_glob;
unsigned long cur_time;
    unsigned long dt ;
    unsigned long dt_glob;
     

void setup(){                            
     Wire.begin();  //80 - enc3, 81 - enc2, 82 - enc1   
     Wire.setClock( 400000UL);                     
     Wire.onReceive(decod_main_i2c);                
     sWire.begin();                            
     Serial.begin(250000);
     //Serial.println("1");
    
     scale.begin(dataPin, clockPin);

     setup_spi_slave();

     cur_time = millis();
     dt = (cur_time - time_measure);

     memset(val, 'k', sizeof(val));
}                                    
int slave_count = 0;  
bool tens_measured = false;       
bool req1_getted = false;    
bool req2_getted = false;    
bool req3_getted = false;    
bool req4_getted = false;     
bool mes1 = false;   
bool mes2 = false;
bool mes3 = false;
bool mes4 = false;        


void loop(){      
    
    //dt = (cur_time - time_measure);
    unsigned long dt_glob = (cur_time - time_measure_glob);


    if(dt_glob>10 && !req1_getted)  {get_request_i2c(I2C_ADDR1, mes_s1,0); req1_getted = true;get_data_enc(); }
    
    if(dt_glob>20 && !req2_getted)  {get_request_i2c(I2C_ADDR2, mes_s2,1); req2_getted = true;get_data_enc(); }
    
    if(dt_glob>30 && !req3_getted)  {     
      get_request_i2c(I2C_ADDR3, mes_s3,2);
      req3_getted = true;get_data_enc();}
    
    if(dt_glob>40 && !req4_getted)  {
      get_request_i2c(I2C_ADDR4, mes_s4,3);
      req4_getted = true;get_data_enc(); }
    
    if(dt_glob>50 && !tens_measured)  {get_data_tens(); tens_measured  = true;get_data_enc(); Serial.println(val);}
    
    if(dt_glob>60 && !mes1) {get_data_enc();mes1 = true; }
    
    if(dt_glob>70 && !mes2) {get_data_enc(); mes2  = true;}
    
    if(dt_glob>80 && !mes3) {get_data_enc();mes3  = true; }
    
    if(dt_glob>90 && !mes4) {get_data_enc(); mes4  = true;} 
    

    
    /*if(dt>10)//every 10ms
    {

        if(slave_count<4){
        switch(slave_count){
          case 0: get_request_i2c(I2C_ADDR1, mes_s1,slave_count); break;
          case 1: get_request_i2c(I2C_ADDR2, mes_s2,slave_count); break;
          case 2: get_request_i2c(I2C_ADDR3, mes_s3,slave_count); break;
          case 3: get_request_i2c(I2C_ADDR4, mes_s4,slave_count); break;
          default: break;
        }
        
      }
      
      slave_count++;
      get_data_enc();      

       time_measure = cur_time;
       if(dt_glob>30)//every 110ms
       {

         get_data_tens();
         
         
         slave_count = 0;

        // Serial.println(val);

         
       }
    }
*/
     idle_spi_slave();

     //if(xmt_pos>0)Serial.println(xmt_pos);
} 

// SPI interrupt routine
ISR(SPI_STC_vect)
{
    if (bReceivingString)
    {
        byte c = SPDR;

        // add to buffer if room
        if (rcv_pos < sizeof buf)
        {
            buf[rcv_pos++] = c;
        }  // end of room available
    }
    else
    {
        SPDR = val[xmt_pos++];
        //xmt_pos++;
    }
}
void send_i2c(char mes[],byte adr)
{
  Wire.beginTransmission(adr); // 
  Wire.write(mes);       // 
 // Serial.println(mes);
  Wire.endTransmission(); 
  //Serial.println("sendi2c_end");
  
}
char rec_i2c[LEN_I2C]; 
int get_request_i2c(int adr, char rec_i2c[],int ind)
{
    char rec_i2c_int[I2C_REC_LEN];
    memset(rec_i2c_int, 'k', sizeof(rec_i2c_int));
    int i_m =0; 
    bool begin_data = false;
    bool end_data = false;
    Wire.requestFrom(adr,I2C_REC_LEN);
    //Serial.println("__________");
   // Serial.println(Wire.available());
    while(Wire.available() && i_m<I2C_REC_LEN) 
    {  
      int b = Wire.read();
      //Serial.println(b);
      i_m++;
      if (b == 98)
      {
        i_m = 0; 
        begin_data = true;            
      }
     
      if(begin_data && !end_data)
      {
        if(b<254)
        {
          rec_i2c_int[i_m+2] = (char)b;
          
          //Serial.print(rec_i2c_int[i_m] );
        }
        else
        {

          //Serial.println("  err ");
          return -1;
        }      
      } 
       if (b == 99)
      {
        end_data = true;       
      }
    }
    
    if(!begin_data || !end_data) return -1;
    //Serial.println("  ok ");
    /*for(int i=0; i<I2C_REC_LEN;i++)
    {
      rec_i2c[i] = rec_i2c_int[i];
    }    
    Serial.print("from ");
    Serial.print(adr);
    Serial.print(": ");
    Serial.println( rec_i2c);*/
    int i = 0;
    for(int j = I2C_REC_LEN*(ind+1); j <I2C_REC_LEN*(ind+2); j++)
    {
      val[j] = rec_i2c_int[i]; i++;
    }    

    
    return 0;
};



void setup_spi_slave()
{
  
    // have to send on master in, *slave out*
    pinMode(PIN_SPI_SCK, INPUT);
    pinMode(PIN_SPI_MOSI, INPUT);
    pinMode(PIN_SPI_MISO, OUTPUT);
    pinMode(PIN_SPI_SS, INPUT_PULLUP); //SS is INPUT for slave device

    // turn on SPI in slave mode
    SPCR |= _BV(SPE);

    // turn on interrupts
    SPCR |= _BV(SPIE);

    rcv_pos = 0;
    process_it = false;

    //use dstrtof to produce string version of float value
    float x = 3.14159;
    //memset(val, 0x0D, sizeof(val));
    xmt_pos = 0;
}


bool prev_low = false;
void idle_spi_slave()
{
  
    if (process_it)
    {
        //Serial.println(rcv_pos);
        //buf[rcv_pos] = 0;
        //Serial.println( buf);
        //rcv_pos = 0;
       // process_it = false;
       //bReceivingString = false;
    }  // end of flag set

    if (digitalRead(PIN_SPI_SS) == HIGH )
    {
     
        xmt_pos = 0; //get ready for next iteration
        SPDR = 0;

        if(prev_low)
        {
          // pinMode(PIN_SPI_MISO, OUTPUT);
          time_measure_glob = millis();
          time_measure = millis();
          prev_low = false;

          tens_measured = false;       
          req1_getted = false;    
          req2_getted = false;    
          req3_getted = false;    
          req4_getted = false;     
          mes1 = false;   
          mes2 = false;
          mes3 = false;
          mes4 = false;  
        }
        cur_time = millis();
        //bReceivingString = false;
       // process_it = false;

    }
    else
    {
      prev_low = true;
      if(!prev_low)
      {
           //pinMode(PIN_SPI_MISO, INPUT);
            
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
     mes_str = "kkkbd"+String(data_tens)+"s"+String(data_lenght)+"c";
     while(mes_str.length()<I2C_REC_LEN) mes_str+="k";

    int i=0;
      for(int j = 0; j <25; j++)
      {
        val[j] = mes_str[i]; i++;
      }   
     
     //mes_str.toCharArray(mes,sizeof(mes));
     //Serial.println(mes);
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
