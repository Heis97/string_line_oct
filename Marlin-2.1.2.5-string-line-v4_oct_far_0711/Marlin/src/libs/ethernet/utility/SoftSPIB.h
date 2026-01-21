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


#ifndef _SOFTSPIB_H
#define _SOFTSPIB_H

#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
#include <WProgram.h>
#endif

#define ETH_SCK_PIN  PB13
#define ETH_SS_PIN   PB12
#define ETH_MISO_PIN PC2
#define ETH_MOSI_PIN PC3

#define ARD_SCK_PIN  PA5

#define ARD_SS_PIN   PA4
#define ARD_MISO_PIN PA6
#define ARD_MOSI_PIN PA7


#include <SPI.h>

class SoftSPIB {
    private:
        uint8_t _cke;
        uint8_t _ckp;
        uint8_t _delay;
        uint16_t _miso;
        uint16_t _mosi;
        uint16_t _sck;
        uint8_t _order;

    public:
        SoftSPIB(uint16_t mosi, uint16_t miso, uint16_t sck);
        void begin();
        void end();
        void setBitOrder(uint8_t);
        void setDataMode(uint8_t);
        void setClockDivider(uint8_t);
        uint8_t transfer(uint8_t);
        uint8_t transfer_2(uint8_t);
        uint16_t transfer16(uint16_t data);
        uint16_t transferBits(uint16_t, uint8_t);

        void spiBegin();
        uint8_t spiRec();
        void spiRead(uint8_t *buf, uint16_t nbyte);
        //void spiRead(const uint8_t *buf, uint16_t nbyte);
        void spiSend(uint8_t data); 
		void spiSendBlock(uint8_t token, const uint8_t *buf);
};
#endif
