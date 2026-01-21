#include "../MarlinCore.h"
#include "string_periphery.h"
#include "temperature.h"

//#define ETHERNET_TCP


#define I2C_REC_LEN 25
#define UDP_PACKET_LEN 35
#define SPI_PACKET_LEN 125

#define ETHERNET_PERIOD_MS 8  //8
/*
#define TEMP_0_CS_PIN                     PF8   // Max31865 CS
  #define TEMP_0_SCK_PIN                    PA5
  #define TEMP_0_MISO_PIN                   PA6
  #define TEMP_0_MOSI_PIN                   PA7
*/
StringPeriphery  string_manager;

#define SPI_TIME_ASK 110//ms
#define RELE_PIN -1
#define RST_W5500 PD7
#define SPI_SS  PB12
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};


#ifdef PRIMARY_PLATE
IPAddress ip(192,168,10,212);//IPAddress ip(192,168,10,212);
unsigned int localPort = 52000; 
uint16_t remote_port = 50000;
#else
IPAddress ip(192,168,10,211);
unsigned int localPort = 52100; 
uint16_t remote_port = 50001;
#endif



EthernetServer server(localPort );
IPAddress remote = IPAddress(192,168,10,3);

String readString = String(30);

     // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


volatile byte rcvbyte = 0;
char rcvbuf[SPI_PACKET_LEN ];
char rcvbuf_udp[UDP_PACKET_LEN ];

float med_fil_tens[TENSOMETR_NUM][MED_FILTR_LEN]{};
long med_fil_len[TENSOMETR_NUM][MED_FILTR_LEN]{};

#define SPI_SOFT2_MOSI PA7
#define SPI_SOFT2_MISO PA6
#define SPI_SOFT2_SCK PA5
#define SPI_SOFT2_CS  PA4
//uint8_t mosi, uint8_t miso, uint8_t sck
SoftSPIB soft_spi_2(SPI_SOFT2_MOSI,SPI_SOFT2_MISO,SPI_SOFT2_SCK);




void StringPeriphery::init()
{
    
   #ifdef PRIMARY_PLATE

   pinMode(TEMP_0_CS_PIN,OUTPUT);
    pinMode(TEMP_1_CS_PIN,OUTPUT);
    pinMode(TEMP_2_CS_PIN,OUTPUT);

    WRITE(TEMP_0_CS_PIN,HIGH);
    WRITE(TEMP_1_CS_PIN,HIGH);
    WRITE(TEMP_2_CS_PIN,HIGH);

    max6675_temp_cam_ext.begin();
    max6675_temp_cam_intern_1.begin();
    max6675_temp_cam_intern_2.begin();

    pinMode(RELAY_0_PIN,OUTPUT);
    pinMode(RELAY_1_PIN,OUTPUT);
    pinMode(RELAY_2_PIN,OUTPUT);
    pinMode(RELAY_3_PIN,OUTPUT);
    pinMode(RELAY_4_PIN,OUTPUT);
    pinMode(RELAY_5_PIN,OUTPUT);

    WRITE(RELAY_0_PIN,0);
    WRITE(RELAY_1_PIN,0);
    WRITE(RELAY_2_PIN,0);
    WRITE(RELAY_3_PIN,0);

    WRITE(RELAY_4_PIN,0);
    WRITE(RELAY_5_PIN,0);
    //analog_inp.begin();
    //analog_inp.setGain(0);

    mcp4725_hv_v.begin();
    mcp4725_hv_v.setValue(0);

    mcp4725_press.begin();
    mcp4725_press.setValue(0);

    pcf_led1.begin(0x20);
    pcf_led2.begin(0x21);

    pcf_led2.pinMode(3,OUTPUT);

    pcf_led1.digitalWrite(0,LOW);
    pcf_led1.digitalWrite(1,LOW);

    pcf_led1.digitalWrite(2,HIGH);

    pcf_led1.digitalWrite(3,HIGH);

    DELAY_US(100000);
    //set_24v_out(1);
 #else
    mcp4725_turbo.begin();
    mcp4725_turbo.setValue(0);

    SET_OUTPUT(LED_MC1_PIN);
    SET_OUTPUT(LED_MC2_PIN);
    WRITE(LED_MC1_PIN,LOW);
    WRITE(LED_MC2_PIN,LOW);

    SET_OUTPUT(VIBRO1_PIN);
    WRITE(VIBRO1_PIN,LOW) ;
    SET_OUTPUT(VIBRO2_PIN);
    WRITE(VIBRO2_PIN,LOW) ;

    SET_OUTPUT(LED_POUND);
    WRITE(LED_POUND,HIGH) ;

    SET_OUTPUT(FAN4_PIN);
    WRITE(FAN4_PIN,LOW) ;

    motors.setVel(recuperator_def_vel,recuperator_axis);
    /*WRITE(VIBRO1_PIN,HIGH) ;

    delay(1000);

    WRITE(VIBRO1_PIN,LOW) ;
    delay(1000);


        delay(1000);*/
     /*hal.set_pwm_duty(pin_t(VIBRO1_PIN), 10);
     delay(1000);
     hal.set_pwm_duty(pin_t(VIBRO1_PIN), 0);
     delay(1000);
     hal.set_pwm_duty(pin_t(VIBRO1_PIN), 255);
     delay(1000);
     hal.set_pwm_duty(pin_t(VIBRO1_PIN), 0);*/
 #endif

    
    string_spi_begin();

    #ifdef ETHERNET_TCP
        string_tcp_ethernet_begin();
    #else
        string_ethernet_begin_3();
    #endif
    

    //


  
   /* motors.gotopos(1000000000l,X_AXIS);
    motors.setDivider(100,0);
    motors.gotopos(1000000000l,Y_AXIS);
    motors.setDivider(200,1);*/

};


void StringPeriphery::string_spi_begin()
{
  soft_spi_2.begin();
  soft_spi_2.setBitOrder(MSBFIRST);
  soft_spi_2.setDataMode(SPI_MODE2);
  pinMode(SPI_SOFT2_CS,OUTPUT);
   pinMode(SPI_SOFT2_MISO,INPUT);
  memset(rcvbuf, 'k', sizeof(rcvbuf));
};
void StringPeriphery::test_spi_loop_part1(uint8_t num)
{
    //char c = 54;

    /*// send test string
    Serial.println("Sending Test String");

    //tell slave to get ready
    WRITE(SPI_SOFT2_CS, LOW);    // SS is pin 10
    //DELAY_US(200); //give slave some time 

    // send test string
 
    Serial.println(num);
     soft_spi_2.transfer(num);
    soft_spi_2.transfer(num-1);
    soft_spi_2.transfer(num-2);
    soft_spi_2.transfer('\n');
   
        //DELAY_US(20); //required for arduino-arduino xfrs*/
    //char c;

    // send test string
    //Serial.println("Sending Test String");

    //tell slave to get ready
     //give slave some time 

    // send test string
   /* for (const char* p = "Hello, world!\n"; c = *p; p++)
    {
        //Serial.println(c);
        soft_spi_2.transfer_2(c);
        DELAY_NS(8000); //required for arduino-arduino xfrs
    }*/
    //DELAY_US(10); 
    //WRITE(SPI_SOFT2_CS, HIGH);    // SS is pin 10
    //DELAY_US(200); 
    //receive dat

};

void StringPeriphery::string_spi_loop()
{

    //Serial.println("spi loop");
    #ifdef PRIMARY_PLATE
    WRITE(SPI_SOFT2_CS, LOW);  
    DELAY_US(300);
    char c;
    memset(rcvbuf,'k', sizeof(rcvbuf));
    c = 'k';

    /*for(int i=0; i<SPI_PACKET_LEN;i++)
    {
       Serial.print(rcvbuf[i]);
    }
    Serial.println("->rcvbuf_before");*/



    for(int i=0; i<SPI_PACKET_LEN;i++)
    {
        c = (char)soft_spi_2.transfer_2(0);
        rcvbuf[i] = c;
        DELAY_NS(4000);
    }
    DELAY_US(30);
    WRITE(SPI_SOFT2_CS, HIGH);

    /*for(int i=0; i<SPI_PACKET_LEN;i++)
    {
       Serial.print(rcvbuf[i]);
    }
    Serial.println("->rcvbuf");*/

    //--------------------------------------------------------------------
    for(int i=0; i< TENSOMETR_NUM;  i++)
    {
            float force_one;
            long string_len_one;
            parse_data_ts(rcvbuf,&force_one,&string_len_one,force_k[i],25*i);


           // force_cur[i] = force_one- force_off[i];
            //string_lenght[i] = string_len_one;
            
            /*refresh_val(force_one,i,med_fil_tens);
            refresh_val(string_len_one,i,med_fil_len);
            float force_orig = force_k[i]*( med_filtr(i,med_fil_tens)- force_off[i]) ;
            force_cur[i] -= koef_gauss[i]*(force_cur[i]-force_orig);  
            string_lenght[i] = med_filtr(i,med_fil_len);*/


            refresh_val(force_one,i,med_fil_tens);
            refresh_val(string_len_one,i,med_fil_len);
            force_cur[i] = med_filtr(i,med_fil_tens)- force_off[i] ;
            string_lenght[i] = med_filtr(i,med_fil_len);
    }
    #endif
};

void StringPeriphery::refresh_val(float val, int row,float arr[TENSOMETR_NUM][MED_FILTR_LEN])
{
    for(int i=1; i<MED_FILTR_LEN; i++)
    {
        arr[row][i-1] = arr[row][i];
    }
    arr[row][MED_FILTR_LEN-1] = val;

}

float StringPeriphery::med_filtr(int row,float arr[TENSOMETR_NUM][MED_FILTR_LEN])
{
    float v1 = arr[row][0];
    float v2 = arr[row][1];
    float v3 = arr[row][2];

    if(v2>v3 )
    {
        if(v1>v2)
        {
            return v2;
        }
        else//v1<v2
        {
            if(v3>v1)
            {
                return v3;
            }
            else
            {
                return v1;
            }
        }
    }
    //-------------------------------------
    //v2<v3
    else
    {
        if(v1>v3)
        {
            return v3;
        }
        else//v1<v3
        {
            if(v2>v1)
            {
                return v2;
            }
            else//v2<v1
            {
                return v1;
            }
        }
    }

    return v2;
}

void StringPeriphery::refresh_val(long val, int row,long arr[TENSOMETR_NUM][MED_FILTR_LEN])
{
    for(int i=1; i<MED_FILTR_LEN; i++)
    {
        arr[row][i-1] = arr[row][i];
    }
    arr[row][MED_FILTR_LEN-1] = val;

}

long StringPeriphery::med_filtr(int row,long arr[TENSOMETR_NUM][MED_FILTR_LEN])
{
    long v1 = arr[row][0];
    long v2 = arr[row][1];
    long v3 = arr[row][2];

    if(v2>v3 )
    {
        if(v1>v2)
        {
            return v2;
        }
        else
        {
            if(v3>v1)
            {
                return v3;
            }
            else
            {
                return v1;
            }
        }
    }
    //-------------------------------------
    //v2<v3
    else
    {
        if(v1>v3)
        {
            return v3;
        }
        else//v1<v3
        {
            if(v2>v1)
            {
                return v2;
            }
            else//v2<v1
            {
                return v1;
            }
        }
    }

    return v2;
}



void StringPeriphery::string_tcp_ethernet_begin()
{
    pinMode(RST_W5500,OUTPUT);
  
  WRITE(RST_W5500,1);
  delay(200);
  
  WRITE(RST_W5500,0);
  delay(20);

  WRITE(RST_W5500,1);
  delay(200);
  
  Ethernet.init(SPI_SS);

  Ethernet.begin(mac, ip);
  server.begin();


  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  pinMode(SPI_SS,OUTPUT);
  pinMode(RELE_PIN,OUTPUT);

};

void StringPeriphery::string_ethernet_begin_3()
{
    pinMode(RST_W5500,OUTPUT);
  
  WRITE(RST_W5500,1);
  delay(200);
  
  WRITE(RST_W5500,0);
  delay(20);

  WRITE(RST_W5500,1);
  delay(200);
  
  Ethernet.init(SPI_SS);
  Ethernet.begin(mac, ip);

  w5500.setPHYCFGR(255);
  Udp.begin(localPort);
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
   
  pinMode(SPI_SS,OUTPUT);
  pinMode(RELE_PIN,OUTPUT);

};

void StringPeriphery::test_ethernet_loop()
{
   EthernetClient client = server.available();
   //Serial.print("IP: ");
 // Serial.println(Ethernet.localIP());
  if (client) {
   // Serial.println("new client");
 
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
 
        if (readString.length() < 30){readString.concat(c);}Serial.print(c);
 
        if (c == '\n') {
 
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println("Refresh: 5"); // время обновления страницы 
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html><meta charset='UTF-8'>");
 
          client.println("<h1>Реле: ");
 
        if(readString.indexOf("p=1") >=0){client.println("ON");WRITE(RELE_PIN,HIGH);}
        else if(readString.indexOf("p=0") >=0){client.println("OFF");WRITE(RELE_PIN,LOW);}
        else{client.println("OFF");WRITE(RELE_PIN,LOW);}
 
          client.println("<h1>");
          client.print("<input type=button value='ON' onmousedown=location.href='/?p=1'> ");
          client.println(" <input type=button value='OFF' onmousedown=location.href='/?p=0'><br/><br/>");
 
          client.println("</html>");
          readString="";
 
          break;
 
        }
      }
    }
 
    //delay(1);
    client.stop();
    //Serial.println("client disconnected");
  }
};


bool ethernet_connected = false;
int count_loop = 0;
void StringPeriphery::string_tcp_ethernet_loop_2()
{
    count_loop++;

   //if(!ethernet_connected)
   {
    client = server.available(); 
   // ethernet_connected = true;
    //Serial.println("client");
    }
   
   //Serial.print("IP: ");
   //Serial.println(Ethernet.localIP());
    //server.println();
    #ifdef PRIMARY_PLATE

   server.write(state_cur().c_str());

   //server.write(rcvbuf);
   #else
   server.write(state_cur_sup().c_str());
   #endif

   if (client) 
   { 
    //Serial.println("new client");
        
    
   
    if(client.connected()) 
    //if(ethernet_connected)
    {//divide commands

        

         memset(rcvbuf_udp, 0, sizeof(rcvbuf_udp));
         
        for(int i=0 ; i< UDP_PACKET_LEN - 1 && client.available(); i++)
        {

            rcvbuf_udp[i] = (char) client.read();
        }

         /*for(int i=0; i< UDP_PACKET_LEN - 1;i++)
        {
            Serial.print(rcvbuf_udp[i]);
        }
        Serial.println();*/
        
         int retq = -1;
        retq = queue.get_serial_commands(rcvbuf_udp,UDP_PACKET_LEN);
        if (retq>=0)
        {

          parser.parse(rcvbuf_udp);
          gcode.process_parsed_command();
        }

    }

    //delay(1);
    //client.stop();

    
    //Serial.println("client disconnected");
  }
};


bool connection_udp = false;

void StringPeriphery::string_ethernet_loop_3() {
   // Serial.println("eth loop");
  int packetSize = Udp.parsePacket();
    if (packetSize)
    {
        memset(rcvbuf_udp, 0, sizeof(rcvbuf_udp));
        if( Udp.read(rcvbuf_udp,sizeof(rcvbuf_udp))>0)
        {
            
            //for(int i=0; i< UDP_PACKET_LEN - 1;i++)
            {
                //Serial.print(rcvbuf_udp[i]);
            }
           // Serial.println();
            
            int retq = -1;
            retq = queue.get_serial_commands(rcvbuf_udp,UDP_PACKET_LEN);
            if (retq>=0)
            {
                parser.parse(rcvbuf_udp);
                gcode.process_parsed_command();
            }
        }
    }

    #ifdef PRIMARY_PLATE
    Udp.beginPacket(remote,remote_port);
    Udp.write(state_cur().c_str());
    Udp.endPacket();
    #else
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.beginPacket(remote, remote_port);
    Udp.write(state_cur_sup().c_str());
    Udp.endPacket();

    #endif
    
  
}


float StringPeriphery::get_vel(){return 0;};

int16_t StringPeriphery::get_pos()
{int16_t pos = 0;
    //pos  = as5600_lin.readAngle();
    //Serial.println(pos);
    return pos;
};
void StringPeriphery::set_vel_lin(float v){};
void StringPeriphery::set_vel_powder(float v){};
void StringPeriphery::set_vel_gateway(float v){};
void StringPeriphery::set_turbine(float v){};
float StringPeriphery::get_temp_cam_ext()
{
    float val = max6675_temp_cam_ext.getTemperature();
    //Serial.println(val);
    return 0;
};

float StringPeriphery::get_temp_cam_intern1()
{
    float val = max6675_temp_cam_intern_1.getTemperature();
    return 0;
};

float StringPeriphery::get_temp_cam_intern2()
{
    float val = max6675_temp_cam_intern_2.getTemperature();

    return 0;
};
float StringPeriphery::get_v_hv()
{
    uint16_t val = analog_inp.readADC(1);
    return val;
};
float StringPeriphery::get_a_hv()
{
    uint16_t val = analog_inp.readADC(2);
    return val;
};
float StringPeriphery::get_v_press()
{
    uint16_t val = analog_inp.readADC(0);
    return val;
};


//----------------------------------------------------
bool swap = false;
float cur_steps_y = 200;

bool def_set = false;

bool spi_chane = true;


uint8_t count_spi = 0;
uint8_t count_data_spi = 0;

unsigned long time_measure_spi = 0;

unsigned long time_measure_test_loop =  0;
unsigned long time_measure_bunk_vibro =  0;
unsigned long time_measure_m_vibro =  0;

unsigned long time_measure_start_led =  0;
unsigned long time_measure_stop_led =  0;

bool phase_test = true;


int test_loop_len = 10;
int count_vibro_wr = 0;


bool bunk_vibro = 1;
unsigned long bunk_vibro_time_relax = 1000;
unsigned long bunk_vibro_time_work =  1000;
long bunk_vibro_time_counter = 0;

bool vibro_set = false;
bool unvibro_set = false;

bool led_up = false;



void StringPeriphery::idle()
{

    manage_motion();
    unsigned long cur_time = millis();
    unsigned long dt = (cur_time- time_measure);
    unsigned long dt_spi =  (cur_time - time_measure_spi);

    unsigned long dt_test_loop =  (cur_time - time_measure_test_loop);
    unsigned long dt_bunk_vibro =  (cur_time - time_measure_bunk_vibro);
    

  
    if(dt>ETHERNET_PERIOD_MS)
    {
        time_measure = cur_time;

         #ifdef ETHERNET_TCP
         
            string_tcp_ethernet_loop_2();
         #else
             string_ethernet_loop_3();

             //Serial.println(w5500.getPHYCFGR(),BIN);

         #endif
       
        
    }


    
    #ifdef PRIMARY_PLATE

    if(dt_spi>SPI_TIME_ASK)
     {
        string_spi_loop();
        time_measure_spi = cur_time;

         //mcp4725_press.setValue(1000);
     }

    unsigned long dt_temp = (cur_time - time_measure_temp);

    //--------------------------------------------------------
    
    if(dt_temp>period_manage_ms  )
    {
       //-----------------------------------
        pressure = get_v_press();
        
        //HV = get_v_hv();// mcp4725_hv_v.getValue();
        max6675_temp_cam_ext.read(); 
        max6675_temp_cam_intern_1.read(); 
        max6675_temp_cam_intern_2.read(); 
        temp_val_int1 = max6675_temp_cam_intern_1.getTemperature();
        temp_val_int2 = max6675_temp_cam_intern_2.getTemperature();
        temp_val_ext = max6675_temp_cam_ext.getTemperature();
        
        manage_heat_duty();
        time_measure_temp = cur_time;
        
        comp_speeds_string();

    }
    

    unsigned long dt_start_led =  (cur_time - time_measure_start_led);
    unsigned long dt_stop_led =  (cur_time - time_measure_stop_led);


     /*if(dt_start_led<2500)
     {
        if(!led_up){
            pcf_led1.digitalWrite(3,HIGH); 
            pcf_led2.digitalWrite(3,LOW); 
            led_up = true;
        }
     }
     if(dt_start_led<5000 && dt_start_led>2500)
     {
        if(led_up){
            pcf_led1.digitalWrite(3,LOW); 
            pcf_led2.digitalWrite(3,HIGH); 
            led_up = false;}
     }
     if(dt_start_led>5000 )
     {
        time_measure_start_led = cur_time;
     }*/



#else

if(bunk_vibro==1)
{
    if(dt_bunk_vibro < bunk_vibro_time_work && dt_bunk_vibro>0)
    {
        
        if(!vibro_set)
        {
          motors._vibro[feed_pound_axis] = 1;
          motors.setVel(vibro_vel,feed_pound_axis);

          
          motors._vibro[gateway_axis] = 1;
          motors.setVel(vibro_vel,gateway_axis);


          vibro_set = true;

            //WRITE(VIBRO1_PIN,HIGH);
            //hal.set_pwm_duty(pin_t(VIBRO1_PIN), 100);
        }
        

       // motors._vibro[gateway_axis] = 1;
       // motors.setVel(100,gateway_axis);

       //Serial.println("vibro");
    };
    if(dt_bunk_vibro < bunk_vibro_time_relax+bunk_vibro_time_work && dt_bunk_vibro > bunk_vibro_time_work)
    {
        if(!unvibro_set)
        {
          motors._vibro[feed_pound_axis] = 0;
          motors.setVel(feed_pound_def_vel,feed_pound_axis);


          motors._vibro[gateway_axis] = 0;
          motors.setVel(gateway_def_vel,gateway_axis);


          unvibro_set = true;
          //WRITE(VIBRO1_PIN,LOW);

          //hal.set_pwm_duty(pin_t(VIBRO1_PIN), 0);
        }
            

        //motors._vibro[gateway_axis] = 0;
        //motors.setVel(10,gateway_axis);

        
        // Serial.println("no vibro");
    };
    if(dt_bunk_vibro > bunk_vibro_time_relax+bunk_vibro_time_work)
    {
        //if(vibro_loop_all - vibro_loop_high < 10) {vibro_loop_high = 0;}
        time_measure_bunk_vibro = cur_time;
        //Serial.println("reset");
        vibro_set = false;
        unvibro_set = false;
        
    };




    //bunk_vibro_time_counter++;
}


if(vibro_main==1)
{

   

    count_vibro_wr++;
    if(count_vibro_wr>vibro_loop_high)
    {
        //offs = 255 - ampl
        // cur = i - offs()
        
        vibro_loop_ampl_cur++;
        
        int vibro_cur1 = vibro_loop_ampl_cur - 255 + vibro_loop_ampl; if(vibro_cur1<0) vibro_cur1=0;

        #ifdef MAKET

        int vibro_cur2 = vibro_loop_ampl_cur - 255 + vibro_loop_ampl2; if(vibro_cur2<0) vibro_cur2=0;
        vibro_cur1+=vibro_cur2;
        #endif

        if(vibro_cur1!= 0)
        {
            count_vibro_wr = 0;
            hal.set_pwm_duty(pin_t(VIBRO1_PIN), (uint16_t)vibro_cur1);
            #ifdef MAKET
            hal.set_pwm_duty(pin_t(VIBRO2_PIN), (uint16_t)vibro_cur2);
            #endif
        }   
    }
    if(vibro_loop_ampl_cur > 255)
    {
       vibro_loop_ampl_cur = 0;
    }

}
else
{
    
}


    #endif


    //--------------------------------------------------------
};


int StringPeriphery::get_request_i2c(int adr, char rec_i2c[])
{
    char rec_i2c_int[I2C_REC_LEN];
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
          rec_i2c_int[i_m] = (char)b;
          
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
    for(int i=0; i<I2C_REC_LEN;i++)
    {
      rec_i2c[i] = rec_i2c_int[i];
    }    

    return 0;
    //Serial.println( rec_i2c);
};


void StringPeriphery::parse_data_ts(char _data[], float* force, long* string_len,float _force_k, int start_ind )
{
  int ind_d = 0;
  int ind_s = 0;
  int ind_c = 0;
  String tens_str = "";
  String enc_str = "";
  for(int i = start_ind; i < SPI_PACKET_LEN -2;i++)
  {
   // Serial.print(_data[i]);
    if((int)_data[i]== 100) 
    {
      ind_d = i;
    }
    if((int)_data[i]== 115) 
    {
      ind_s = i;
    }
    if((int)_data[i]== 99) 
    {
      ind_c = i;
      break;
    }
  }  
   /*Serial.print(ind_d);
   Serial.print(" ");
   Serial.print(ind_s);
   Serial.print(" ");
   Serial.print(ind_c);
   Serial.println(" ");*/
  if(ind_s<=ind_d+1){
    // Serial.println("ind_s<=ind_d+1");
     return;} 

    if(ind_s-ind_d>4)
    {
        for(int i=ind_d+1; i<ind_s-3;i++)
        {
            tens_str+=_data[i];
        } 
    }
    else
    {
            tens_str = "0";
    }
  //Serial.println(tens_str);
  if(ind_c<=ind_s+1)
  {
   // Serial.println("ind_c<=ind_s+1");
    return;
  }
   
  for(int i=ind_s+1; i<ind_c;i++)
  {
    enc_str += _data[i];
  } 
  //Serial.println(enc_str);
  long data_f = tens_str.toInt();//!!!!!!!!!!
  *force =  (float)data_f;

  *string_len = enc_str.toInt();
  //Serial.println(tens);
  //Serial.println(enc);
};

int* StringPeriphery::divide_data_parse(char _data[],int inds[])
{
    int data_count = 0;
    int ind_st_cur = 0;
    int ind_mid_cur = 0;
    int ind_end_cur = 0;
    bool start = false;
    bool middle = false;
    bool stop = false;
    for(int i= 1; i<SPI_PACKET_LEN ;i++)
    {
        if((int)_data[i]== 100)  {start = true;     ind_st_cur = i ;}
        if((int)_data[i]== 115)  {middle = true;    ind_mid_cur = i ;}
        if((int)_data[i]== 99)   {stop = true;      ind_end_cur = i ;}

        if(start && middle && stop && ind_st_cur<ind_mid_cur<ind_end_cur && data_count<5)
        {
            inds[data_count] = ind_st_cur-1;

            data_count++;
            start = false;
            middle = false;
            stop = false;

            ind_st_cur = 0;
            ind_mid_cur = 0;
            ind_end_cur = 0;
    
        }
    } 


    return inds;
}


void  StringPeriphery::set_vel_strings(float vel)
{

    for(int i=0; i<TENSOMETR_NUM; i++)
    {
        orig_speed_tens[i] = vel;
    }
    orig_speed_tens_com = vel;
   
};

void  StringPeriphery::reset_vel_strings()
{

    for(int i=0; i<TENSOMETR_NUM; i++)
    {

        koef_tens[i] = 1;
    }
    //orig_speed_tens_com = vel;
   
};




void StringPeriphery::set_hv_v(uint16_t v)
{
    mcp4725_hv_v.setValue(v);
    
};
void StringPeriphery::set_hv_i(uint16_t v)
{
    //mcp4725_hv_i.setValue(v);
};
void StringPeriphery::set_press(uint16_t v)
{
    pressure_dest = v;
     mcp4725_press.setValue(v);

};

void StringPeriphery::set_turbo(uint16_t v)
{
     mcp4725_turbo.setValue(v);
 turbo_val_cur = v;

//Serial.println(turbo_val_cur);


};

void StringPeriphery::set_reley_1(int v)
{
    int v_set = v;
    //if(v==0) v_set = 1;
    //else v_set = 0;
    WRITE(RELAY_0_PIN,v_set);
    reley_1 = v;
    
};
void StringPeriphery::set_reley_2(int v)
{
   int v_set = v;
    //if(v==0) v_set = 1;
    //else v_set = 0;
    WRITE(RELAY_1_PIN,v_set);
    reley_2 = v;
};

void StringPeriphery::set_reley_heater(int ind, int v)
{
    if(ind == 0) set_reley_1(v);
    else if(ind == 1) set_reley_2(v);
};

void StringPeriphery::set_reley_HV(int v)
{
    int v_set = v;
    //if(v==0) v_set = 1;
    //else v_set = 0;
    WRITE(RELAY_2_PIN,v_set);
    reley_HV = v;
};
void StringPeriphery::set_reley_press(int v)
{
   int v_set = v;
    //if(v==0) v_set = 1;
    //else v_set = 0;
    WRITE(RELAY_3_PIN,v_set);
    reley_press = v;
};

void StringPeriphery::set_heater_2(int v)
{
    WRITE(HEATER_2_PIN,v);
    //turbo = v;
};
void StringPeriphery::set_heater_3(int v)
{
    WRITE(HEATER_3_PIN,v);
};

void StringPeriphery::set_24v_out(int v)
{
   WRITE(RELAY_4_PIN,v);
   reley_24_out = v;
}

void StringPeriphery::set_24v_reset(int v)
{
    WRITE(RELAY_5_PIN,v);
}





String StringPeriphery::state_cur()
{
    //long steps = (long)motors.readSteps()[0];
    //bool homed = (bool)motors.readHoming()[0];
    String delim = " ";
    String state = "string: "+ 
    String(temp_val_int1)+ delim+//0
    String(temp_val_int2)+delim+//1
    String(temp_val_ext)+delim+//2
    String(orig_speed_tens_com)+delim+//3  
    String(reley_24_out)+delim+//4
    String(reley_HV)+delim+//5
    String(reley_press)+delim+//6
    String(pressure)+delim+//7
    String(HV)+delim+//8
    String(motors_free_state)+delim+//9  
    String(time_measure)+delim+ //10
    String((int16_t)motors.readHoming_one(karet_axis))+delim+//11  //homing karet
    String(ind_sensor)+delim+//12  
    String(tare_tens_state)+delim+//13  //
     String(pressure_dest)+delim+//14  //
      String(temp_dest)+delim+//15 //
       String(heater_en)+delim;//16 //

    for(byte i=0; i<TENSOMETR_NUM; i++)
    {
            state+=String(string_move_second[i])+delim;//17
            state+=String(force_cur[i])+delim;//18
            state+=String(string_lenght[i])+delim;//19
            state+=String(cur_speed_tens[i])+delim;//20
            state+=String(force_dest[i])+delim;//21
    }
    return state;
};


String StringPeriphery::state_cur_sup()
{
    String state = "str2ing: ";
    #ifndef PRIMARY_PLATE
    String delim = " ";
    state = "str2ing: "+ 
    
    String(turbo_val_cur)+ delim+//0
    String(time_measure)+delim+//1

    String(gateway_move)+delim+//2
    String(feed_pound_move)+delim+//3
    String(recuperator_move)+delim+//4

    String(vibro_main)+delim+//5
    String(vibro_loop_high)+delim+//6
    String(vibro_loop_ampl)+delim+//7

    String(homed_d)+delim+//8 
    String(homed_e)+delim+//9
    String(mirror_h_off_d)+delim+//10
    String(camera_h_off_d)+delim+//11
    String(mirror_h_off_e)+delim+//12
    String(camera_h_off_e)+delim+//13
    String(led_micr_d)+delim+//12
    String(led_micr_e)+delim+//13
    " ";

    #endif
    return state;
    
};
#ifndef PRIMARY_PLATE
void StringPeriphery::set_led_micro_d(uint8_t v)
{
     WRITE(LED_MC1_PIN,v);
     led_micr_d = v;
};
void StringPeriphery::set_led_micro_e(uint8_t v)
{
     WRITE(LED_MC2_PIN,v);
     led_micr_e = v;
};
#endif
void StringPeriphery::report_state()
{
    Serial.println(state_cur());
};


//float temp_dest = 0;
//int ind_sensor = 0;
//int heater_en = 0;

void StringPeriphery::manage_heat()
{
    if(heater_en == 1)
    {
        float cur_temp = 0;
        if(ind_sensor==0)
        {
            cur_temp = temp_val_ext;
            
        }
        else if(ind_sensor==1)
        {
            cur_temp = temp_val_int1;
            
        }
        else if(ind_sensor==2)
        {
            cur_temp = temp_val_int2;
        }
        else
        {
            set_reley_1(0);
            set_reley_2(0);
            return;
        }
        if(cur_temp<temp_dest+temp_hyst)
        {
            set_reley_1(1);
            set_reley_2(1);
        }
        else if(cur_temp>temp_dest-temp_hyst)
        {
            set_reley_1(0);
            set_reley_2(0);
        }
    }
    else
    {
        set_reley_1(0);
        set_reley_2(0);
    }
}


void StringPeriphery::manage_heat_duty()
{
    float temp_1 = temp_val_int1;
    float temp_2 = temp_val_int2;
    if(ind_sensor == 0)
    {
        temp_1 = temp_val_ext;
        temp_2 = temp_val_ext;  
    }
    if(heater_en == 1)
    {
        duty_1 = manage_heat_duty_single(0, temp_1,kp_1);
        duty_2 = manage_heat_duty_single(1, temp_2,kp_2);

        if(duty_1 < 0) heating_1 = false; else heating_1 = true;
        if(duty_2 < 0) heating_2 = false; else heating_2 = true;

        heat_pwm_control();
    }
    else
    {
        set_reley_1(0);
        set_reley_2(0);
    }

    
}

void StringPeriphery::motors_free(int v)
{
    if (v==0)
    {
        for(int i=0; i< TENSOMETR_NUM; i++)
        {
            motors.wake_up(motors_tens[i]);
            
        }
    }
    else
    {
        for(int i=0; i< TENSOMETR_NUM; i++)
        {
            motors.sleep(motors_tens[i]);
        }
    }
    motors_free_state = v;
};

void StringPeriphery::tare_tens(int v)
{


    tare_tens_state = v;
};

int StringPeriphery::manage_heat_duty_single(int ind, float temp,float kp)
{    
    bool heating = false;
    int duty = 0;
    if(temp<temp_dest+temp_hyst)
    {
        
        heating = true;
    }
    else if(temp>temp_dest-temp_hyst)
    {
        heating = false;
    }  
    else
    {
        heating = false;
    }

    if(!heating)
    {
        return -1;
    }

    duty = kp * abs(temp_dest-temp);
    if(duty>cycle_time-5) duty = cycle_time-5;
    

    return duty;
}
void StringPeriphery::heat_pwm_control()
{
    if(heating_1) heat_pwm_control_single(0, duty_counter, duty_1); else  set_reley_1(0);
    if(heating_2) heat_pwm_control_single(1, duty_counter, duty_2); else  set_reley_2(0);
    
    duty_counter++;
    if(duty_counter>cycle_time) 
    {
        duty_counter = 0;       
    }
};
void StringPeriphery::heat_pwm_control_single(int ind, int counter, int duty)
{
    if(counter==0)
    {
        set_reley_heater(ind,1);
    }
    else if(counter == duty)
    {
        set_reley_heater(ind,0);
    }

};



void StringPeriphery::set_heaters_enable(int v)
{
    heater_en = v;
};
void StringPeriphery::set_heaters_temp(float v)
{
    temp_dest = v;
};
void StringPeriphery::set_heaters_ind(int v)
{
    ind_sensor = v;
};
void StringPeriphery::set_reporting(bool state){
    reporting = state;
};


void StringPeriphery::manage_motion()
{
   
    #ifdef PRIMARY_PLATE

        if(karet_move == 1   ) { manage_axis((AxisEnum) karet_axis,dir[karet_axis]);     } else {  if(!motors.readHoming_one(karet_axis)) motors.step(0L,karet_axis);  };

        if(string_move == 1) { manage_axis((AxisEnum) motor_com_axis,dir[motor_com_axis]); } else { motors.step(0L,motor_com_axis);    };
        
        for(int i=0; i< TENSOMETR_NUM; i++)
        {
                if(string_move_second[i] == 1) {manage_axis((AxisEnum)  motors_tens[i] ,dir[motors_tens[i]]);   } else { motors.step(0L,motors_tens[i]);    };
        }
    #else

        //if(feed_pound_move == 1) { manage_axis((AxisEnum) feed_pound_axis,dir[feed_pound_axis]);     } else { motors.step(0L,feed_pound_axis);     };
        if(gateway_move == 1) 
        { 
            if(motors._vibro[gateway_axis]==0)  manage_axis((AxisEnum) gateway_axis,dir[gateway_axis]);
        }
        else{ motors.step(0L,gateway_axis); };
        if(feed_pound_move == 1)            
        {
            if(motors._vibro[feed_pound_axis]==0)   manage_axis((AxisEnum) feed_pound_axis,dir[feed_pound_axis]);
        }
        else { motors.step(0L,feed_pound_axis); };  

        if(recuperator_move == 1) { manage_axis((AxisEnum) recuperator_axis,dir[recuperator_axis]);     } else { motors.step(0L,recuperator_axis);  };
       // if(feed_pound_move == 1) {  k[feed_pound_axis] = manage_axis_vibro_simple((AxisEnum) feed_pound_axis,dir[feed_pound_axis], vibr[feed_pound_axis],k[feed_pound_axis],k_m[feed_pound_axis] );  } else { motors.step(0L,feed_pound_axis);   };
        //if(gateway_move == 1)    {  k[gateway_axis]    = manage_axis_vibro_simple((AxisEnum) gateway_axis,   dir[gateway_axis],    vibr[gateway_axis],   k[gateway_axis],   k_m[gateway_axis]    );  } else { motors.step(0L,gateway_axis);   };

    #endif
    //}
};

int StringPeriphery::manage_axis_vibro(AxisEnum Axis,  int vibr, int k, int k_m,int dir,int _vibr_a){
    int sign = 1;
    if(vibr==1) 
    {
        if(k_m - k <= _vibr_a)
        { 
            sign=-1; 
            k = 0;
        };
    }
    //destination[Axis] = current_position[Axis] + dir*sign*dist_m; 

    motors.step((long int)(dir*steps_m*sign),Axis);
    k++;
    return k;
};

int StringPeriphery::manage_axis_vibro_simple(AxisEnum Axis,int dir,  int vibr, int k, int k_m){
    int k_sign = SIGN (k);
    int k_abs = abs(k);
    
    if(vibr==1 )
    {
        if(k_abs>k_m) 
        {
            k_sign*=-1;
            k_abs = 0;
        }
        k_abs++;
    }
    
    motors.step((long int)(dir*steps_m*k_sign),Axis);
   
    return k_sign *k_abs;
};

void StringPeriphery::manage_axis(AxisEnum Axis, int dir){
    motors.step((long int)(dir*steps_m),Axis);
};



float StringPeriphery::comp_one_tension(float koef_tens, float koef_v_tens, float force_cur, float force_dest,int en)
{
    if(en!=1) return koef_tens;
    float df = force_dest - force_cur;
    koef_tens +=df*koef_v_tens;
    return koef_tens;
};

void StringPeriphery::comp_speeds_string()
{
    for(int i=0; i<TENSOMETR_NUM;i++) {
        koef_tens[i] = comp_one_tension(koef_tens[i],koef_v_tens[i],(float)force_cur[i],force_dest[i],string_move_second[i]);
        cur_speed_tens[i] = koef_tens[i]*orig_speed_tens[i];
        motors.setVel(cur_speed_tens[i],motors_tens[i]);
    }

    motors.setVel(orig_speed_tens_com,motor_com_axis);
}

#ifndef PRIMARY_PLATE


void StringPeriphery::move_one_axis(AxisEnum ax, float dist)
{
    motors.gotopos(dist,ax);
};
void StringPeriphery::move_two_axis(AxisEnum ax1, float dist1,AxisEnum ax2, float dist2)
{
    motors.gotopos(dist1,ax1);
    motors.gotopos(dist2,ax2);
};
//--------------D micro----------------
void StringPeriphery::move_depth_d(float dist)
{
    camera_v_off_d = dist;
    float cam_x = camera_coord_d + camera_v_off_d  + camera_h_off_d;
    apply_limits_d(cam_x,mirror_cur_d);
    move_one_axis(camera_axis_d,  camera_cur_d  );
};
void StringPeriphery::move_betw_string_d(float dist)
{
    camera_h_off_d = dist;
    mirror_h_off_d = dist;
    float cam_x = camera_coord_d + camera_v_off_d + camera_h_off_d;
    float mir_x = mirror_coord_d + mirror_h_off_d;
    apply_limits_d(cam_x, mir_x);
    move_two_axis(camera_axis_d,camera_cur_d,mirror_axis_d, mirror_cur_d);
};
void StringPeriphery::move_to_pos_d(int num)
{
    camera_coord_d = pos_camera_d[num];
    mirror_coord_d = pos_mirror_d[num];
    apply_limits_d(camera_coord_d, mirror_coord_d) ;
    move_two_axis(camera_axis_d,camera_cur_d,mirror_axis_d, mirror_cur_d);
};
void StringPeriphery::apply_limits_d(float _cam_coord_in,float _mir_coord_in)
{
    float _camera_coord = _cam_coord_in;
    float _mirror_coord = _mir_coord_in;
    if (!homed_d)
    {
        return;
    };
    if (_mirror_coord < 0) _mirror_coord = 0;
    if (_camera_coord < 0) _camera_coord = 0;

    if (_mirror_coord > limit_mirror_d) _mirror_coord = limit_mirror_d;
    if (_camera_coord > limit_camera_d) _camera_coord = limit_camera_d;

    if (_mirror_coord + offset_mirror_d < _camera_coord) _camera_coord =_mirror_coord + offset_mirror_d;

    mirror_cur_d = _mirror_coord;
    camera_cur_d = _camera_coord;

};
void StringPeriphery::home_d(uint8_t v)
{
    if(v==1)
    {
        motors.home_axis(camera_axis_d);
        motors.home_axis(mirror_axis_d);
    }
    else
    {

    }
    
    homed_d = v;
};


//--------------E micro----------------


void StringPeriphery::move_depth_e(float dist)
{
    camera_v_off_e = dist;
    float cam_x = camera_coord_e + camera_v_off_e  + camera_h_off_e;
    apply_limits_e(cam_x,mirror_cur_e);
    move_one_axis(camera_axis_e,  camera_cur_e  );
};
void StringPeriphery::move_betw_string_e(float dist)
{
    camera_h_off_e = dist;
    mirror_h_off_e = dist;
    float cam_x = camera_coord_e + camera_v_off_e + camera_h_off_e;
    float mir_x = mirror_coord_e + mirror_h_off_e;
    apply_limits_e(cam_x, mir_x);
    move_two_axis(camera_axis_e,camera_cur_e,mirror_axis_e, mirror_cur_e);
};
void StringPeriphery::move_to_pos_e(int num)
{
    camera_coord_e = pos_camera_e[num];
    mirror_coord_e = pos_mirror_e[num];
    apply_limits_e(camera_coord_e, mirror_coord_e) ;
    move_two_axis(camera_axis_e,camera_cur_e,mirror_axis_e, mirror_cur_e);
};
void StringPeriphery::apply_limits_e(float _cam_coord_in,float _mir_coord_in)
{
    float _camera_coord = _cam_coord_in;
    float _mirror_coord = _mir_coord_in;
    if (!homed_e)
    {
        return;
    };
    if (_mirror_coord < 0) _mirror_coord = 0;
    if (_camera_coord < 0) _camera_coord = 0;

    if (_mirror_coord > limit_mirror_e) _mirror_coord = limit_mirror_e;
    if (_camera_coord > limit_camera_e) _camera_coord = limit_camera_e;

    if (_mirror_coord + offset_mirror_e < _camera_coord) _camera_coord =_mirror_coord + offset_mirror_e;

    mirror_cur_e = _mirror_coord;
    camera_cur_e = _camera_coord;

};
void StringPeriphery::home_e(uint8_t v)
{
    if(v==1)
    {
        motors.home_axis(camera_axis_e);
        motors.home_axis(mirror_axis_e);
    }
    else
    {
        
    }
    
    homed_e = v;
};
#endif


