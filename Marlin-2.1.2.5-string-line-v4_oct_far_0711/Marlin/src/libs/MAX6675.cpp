//
//    FILE: MAX6675.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.3
// PURPOSE: Arduino library for MAX6675 chip for K type thermocouple
//    DATE: 2022-01-11
//     URL: https://github.com/RobTillaart/MAX6675


#include "MAX6675.h"

#include "ethernet/utility/SoftSPIB.h"


// SW SPI
MAX6675::MAX6675(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk)
{
  _select = spi_cs;
  _miso   = spi_miso;
  _clock  = spi_clk;

}


void MAX6675::begin()
{
  _lastTimeRead = 0;
  _offset       = 0;
  _status       = STATUS_NOREAD;
  _temperature  = MAX6675_NO_TEMPERATURE;
  _rawData      = 0;

  pinMode(_select, OUTPUT);
  WRITE(_select, HIGH);
  pinMode(_clock, OUTPUT);
  WRITE(_clock, LOW);
  pinMode(_miso, INPUT);


  
}

uint8_t MAX6675::read()
{
  //  return value of _read()  page 5 datasheet
  //  BITS       DESCRIPTION
  //  ------------------------------
  //       00    three state ?
  //       01    device ID ?
  //       02    INPUT OPEN
  //  03 - 14    TEMPERATURE (RAW)
  //       15    SIGN
  uint16_t value = (uint16_t)_read();

  //  needs a pull up on MISO pin to work properly!
  if (value == 0xFFFF)
  {
    _status = STATUS_NO_COMMUNICATION;
    return _status;
  }

  _lastTimeRead = millis();

  //  process status bit 2
  _status = value & 0x04;

   value >>= 3;

  //  process temperature bits
  _temperature = (value & 0x1FFF) * 0.25;
  //  dummy negative flag set ?
  //  if (value & 0x2000)
  return _status;
}


///////////////////////////////////////////////////
//
//  PRIVATE
//
uint32_t MAX6675::_read(void)
{
  uint32_t _rawData = 0;
  //  DATA TRANSFER


  WRITE(_select, LOW);
  cli();
  DELAY_NS_VAR(200);
  
  for (int8_t i = 15; i >= 0; i--)
  {
    _rawData <<= 1;
    WRITE(ARD_SCK_PIN, HIGH);//rt

    DELAY_NS_VAR(50);
    if (READ(ARD_MISO_PIN)) _rawData++;
    DELAY_NS_VAR(50);
    WRITE(ARD_SCK_PIN, LOW);
    DELAY_NS_VAR(100);  
  }
  sei();
  //DELAY_NS_VAR(1800);
  WRITE(_select, HIGH);
  

  return _rawData;
}

