/*
 * Copyright (c) 2014, Majenko Technologies
 * Copyright (c) 2020, Andriy Golovnya
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#ifdef __AVR__

/*
#define SPI_SOFT_MOSI PC3
#define SPI_SOFT_MISO PC2
#define SPI_SOFT_SCK PB13
#define SPI_SOFT_CS  PB12
*/




#include "../../../inc/MarlinConfig.h"

#define nop asm volatile ("\tnop\n")
#include "SoftSPIB.h"

SoftSPIB::SoftSPIB(uint16_t mosi, uint16_t miso, uint16_t sck) {
    _mosi = mosi;
    _miso = miso;
    _sck = sck;
    _delay = 2;
    _cke = 0;
    _ckp = 0;
    _order = MSBFIRST;
}

void SoftSPIB::begin() {
   /* pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);//INPUT
    pinMode(_sck, OUTPUT);    
*/
  SET_OUTPUT(ETH_SCK_PIN);
  SET_INPUT( ETH_MISO_PIN);
  SET_OUTPUT( ETH_MOSI_PIN);

  SET_OUTPUT(ARD_SCK_PIN);
  SET_INPUT( ARD_MISO_PIN);
  SET_OUTPUT( ARD_MOSI_PIN);
    spiBegin();
}

void SoftSPIB::end() {
   // pinMode(_mosi, OUTPUT);
    //pinMode(_miso, INPUT);
    //pinMode(_sck, OUTPUT);


    SET_OUTPUT(ETH_SCK_PIN);
  SET_INPUT( ETH_MISO_PIN);
  SET_OUTPUT( ETH_MOSI_PIN);

  SET_OUTPUT(ARD_SCK_PIN);
  SET_INPUT( ARD_MISO_PIN);
  SET_OUTPUT( ARD_MOSI_PIN);
}

void SoftSPIB::setBitOrder(uint8_t order) {
    _order = order & 1;
}

void SoftSPIB::setDataMode(uint8_t mode) {
    switch (mode) {
        case SPI_MODE0:
            _ckp = 0;
            _cke = 0;
            break;
        case SPI_MODE1:
            _ckp = 0;
            _cke = 1;
            break;
        case SPI_MODE2:
            _ckp = 1;
            _cke = 0;
            break;
        case SPI_MODE3:
            _ckp = 1;
            _cke = 1;
            break;
    }
}

void SoftSPIB::setClockDivider(uint8_t div) {
    if (div == SPI_CLOCK_DIV2) _delay = 2;
    else if (div == SPI_CLOCK_DIV4) _delay = 4;
    else if (div == SPI_CLOCK_DIV8) _delay = 8;
    else if (div == SPI_CLOCK_DIV16) _delay = 16;
    else if (div == SPI_CLOCK_DIV32) _delay = 32;
    else if (div == SPI_CLOCK_DIV64) _delay = 64;
    else if (div == SPI_CLOCK_DIV128) _delay = 128;
    else _delay = 128;
}

uint8_t SoftSPIB::transfer(uint8_t val) {
    uint8_t out = 0;
    if (_order == MSBFIRST) {
        uint8_t v2 = 
            ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7);
        val = v2;
    }

    uint8_t del = _delay >> 1;

    uint8_t bval = 0;
    cli();
    for (uint8_t bit = 0; bit < 8; bit++) 
    {

        WRITE(_sck, LOW);
        nop; // adjust so SCK is nice
        nop;
        WRITE(_mosi, val & (1<<bit) ? HIGH : LOW);
        WRITE(_sck, _ckp ? HIGH : LOW);

        bval =  READ(_miso);

        out <<= 1;
        out |= bval;
    }
        

       // for (uint8_t i = 0; i < del; i++) {
        //    asm volatile("nop");
       // }
    sei();
    return out;
}

/*uint16_t SoftSPIB::transfer16_old(uint16_t data)
{
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} in, out;
  
	in.val = data;

	if ( _order == MSBFIRST ) {
		out.msb = transfer(in.msb);
		out.lsb = transfer(in.lsb);
	} else {
		out.lsb = transfer(in.lsb);
		out.msb = transfer(in.msb);
	}

	return out.val;
}

uint8_t SoftSPIB::transfer_old(uint8_t val) {
    uint8_t out = 0;
    if (_order == MSBFIRST) {
        uint8_t v2 = 
            ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7);
        val = v2;
    }

    uint8_t del = _delay >> 1;

    uint8_t bval = 0;
    for (uint8_t bit = 0; bit < 8; bit++) {

        //if( _ckp) gio::low(_sck);  else  gio::high(_sck); 
        digitalWrite(_sck, _ckp ? LOW : HIGH);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        if (_cke) {
            bval = digitalRead(_miso);
            //bval = gio::read(_miso);
            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << 7;
            }
        } else {
            //if( val & (1<<bit)) gio::high(_mosi);  else  gio::low(_mosi); 

            digitalWrite(_mosi, val & (1<<bit) ? HIGH : LOW);
        }

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }
            
        //if( _ckp) gio::high(_sck);  else  gio::low(_sck); 
        digitalWrite(_sck, _ckp ? HIGH : LOW);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        if (_cke) {
            //if( val & (1<<bit)) gio::high(_mosi);  else  gio::low(_mosi); 
            digitalWrite(_mosi, val & (1<<bit) ? HIGH : LOW);
        } else {
            bval = digitalRead(_miso);
           // bval = gio::read(_miso);
            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << 7;
            }
        }

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }
    }
    return out;
}

uint16_t SoftSPIB::transferBits_old(uint16_t val, uint8_t bits) {
    uint16_t out = 0;
    if (_order == MSBFIRST) {
        uint16_t v2 = 0;
        for (uint8_t bit = 0u; bit < bits; bit++)
            v2 |= ((val >> bit) & 0x0001) << (bits - bit - 1);
        val = v2;
    }

    uint8_t del = _delay >> 1;

    uint16_t bval = 0;
    for (uint8_t bit = 0; bit < bits; bit++) {

        //if( _ckp) gio::low(_sck);  else  gio::high(_sck); 
        digitalWrite(_sck, _ckp ? LOW : HIGH);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        if (_cke) {
            bval = digitalRead(_miso);

            //bval = gio::read(_miso); //digitalRead(_miso);
            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << (bits - 1);
            }
        } else {
             //if( val & (1<<bit)) gio::high(_mosi);  else  gio::low(_mosi); 
            digitalWrite(_mosi, (val >> bit) & 1 ? HIGH : LOW);
        }

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }
        //if( _ckp) gio::high(_sck);  else  gio::low(_sck); 
        digitalWrite(_sck, _ckp ? HIGH : LOW);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        if (_cke) {
            // if( val & (1<<bit)) gio::high(_mosi);  else  gio::low(_mosi); 
            digitalWrite(_mosi, (val >> bit) & 1 ? HIGH : LOW);
        } else {
            bval = digitalRead(_miso);
             //bval = gio::read(_miso);
            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << (bits - 1);
            }
        }

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }
    }
    return out;
}


*/

void SoftSPIB::spiBegin() {
  //#if PIN_EXISTS(SD_SS)
    // Do not init HIGH for boards with pin 4 used as Fans or Heaters or otherwise, not likely to have multiple SPI devices anyway.
    #if defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__)
      // SS must be in output mode even it is not chip select
      SET_OUTPUT(SD_SS_PIN);
    #else
      // set SS high - may be chip select for another SPI device
      OUT_WRITE(SD_SS_PIN, HIGH);
    #endif
  //#endif
  //SET_OUTPUT(SD_SCK_PIN);
  //SET_INPUT(SD_MISO_PIN);
  //SET_OUTPUT(SD_MOSI_PIN);

  SET_OUTPUT(ETH_SCK_PIN);
  SET_INPUT( ETH_MISO_PIN);
  SET_OUTPUT( ETH_MOSI_PIN);

  SET_OUTPUT(ARD_SCK_PIN);
  SET_INPUT( ARD_MISO_PIN);
  SET_OUTPUT( ARD_MOSI_PIN);


  //spiInit(SPI_HALF_SPEED);
}


 // Soft SPI receive byte
  uint8_t SoftSPIB::spiRec() {
    uint8_t data = 0;
    // no interrupts during byte receive - about 8µs
    cli();
    // output pin high - like sending 0xFF
    WRITE(ETH_MOSI_PIN, HIGH);

    for (uint8_t i = 0; i < 8; ++i) {
      WRITE(ETH_SCK_PIN, HIGH);

      nop; // adjust so SCK is nice
      nop;

      data <<= 1;

      if (READ(ETH_MISO_PIN)) data |= 1;

      WRITE(ETH_SCK_PIN, LOW);
    }

    sei();
    return data;
  }

  // Soft SPI read data
  void SoftSPIB::spiRead(uint8_t *buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }
 /* void SoftSPIB::spiRead(const uint8_t *buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }*/
  // Soft SPI send byte
  void SoftSPIB::spiSend(uint8_t data) {
    // no interrupts during byte send - about 8µs
    cli();
    for (uint8_t i = 0; i < 8; ++i) {
      WRITE(ETH_SCK_PIN, LOW);
      WRITE(ETH_MOSI_PIN, data & 0x80);
      data <<= 1;
      WRITE(ETH_SCK_PIN, HIGH);
    }

  //  nop; // hold SCK high for a few ns
   // nop;
   // nop;
   // nop;

    WRITE(ETH_SCK_PIN, LOW);

    sei();
  }


uint8_t SoftSPIB::transfer_2(uint8_t data_out) {

     uint8_t data = 0;

    cli();
    // output pin high - like sending 0xFF
    //WRITE(ARD_MOSI_PIN, HIGH);

    for (uint8_t i = 0; i < 8; ++i) {
      WRITE(ARD_SCK_PIN, LOW);

      nop; // adjust so SCK is nice
      nop;
      WRITE(ARD_MOSI_PIN, data_out & 0x80);
      data <<= 1;
      data_out <<= 1;
      if (READ(ARD_MISO_PIN)) data |= 1;

      WRITE(ARD_SCK_PIN, HIGH);
      nop; // adjust so SCK is nice
      nop;

    }
    WRITE(ARD_SCK_PIN, LOW);
    WRITE(ARD_MOSI_PIN, HIGH);
    sei();
    return data;
  }
  // Soft SPI send block
  void SoftSPIB::spiSendBlock(uint8_t token, const uint8_t *buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++)
      spiSend(buf[i]);
  }
