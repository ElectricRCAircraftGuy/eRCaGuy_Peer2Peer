/*
eRCaGuy_Peer2Peer

AUTHOR:
By Gabriel Staples
Website: http://www.ElectricRCAircraftGuy.com
My contact info is available by clicking the "Contact Me" tab at the top of my website.
Library Written: 30 May 2017  
Library Last Updated: 30 May 2017 

VERSIONING SYSTEM: 
-Using Semantic Versioning 2.0.0 (http://semver.org/)

Current Library Version: see History section below.

HISTORY (newest on BOTTOM):
20170530 - v0.1.0 - first version [IN WORK]
 
*/

/*
===================================================================================================
  LICENSE & DISCLAIMER
  Copyright (C) 2016 Gabriel Staples.  All right reserved.
  
  This file is part of eRCaGuy_Peer2Peer.
  
  I AM WILLING TO DUAL-LICENSE THIS SOFTWARE--EX: BY SELLING YOU A SEPARATE COMMERICAL LICENSE FOR
  PRORPRIETARY USE. HOWEVER, UNLESS YOU HAVE PAID FOR AND RECEIVED A RECEIPT FOR AN ALTERNATE 
  LICENSE AGREEMENT, FROM ME, THE COPYRIGHT OWNER, THIS SOFTWARE IS LICENSED UNDER THE GNU GPLV3
  OR LATER, A COPY-LEFT LICENSE, AS FOLLOWS.
  
  NB: THE GNU GPLV3 LICENSE IS AN OPEN SOURCE LICENSE WHICH REQUIRES THAT ALL DERIVATIVE WORKS 
  YOU CREATE (IE: *ANY AND ALL* CODE YOU HAVE LINKING TO, BORROWING FROM, OR OTHERWISE USING THIS CODE) 
  ALSO BE RELEASED UNDER THE SAME LICENSE, AND BE OPEN-SOURCE FOR YOUR USERS AND/OR CUSTOMERS.
  FOR ALL STIPULATIONS AND LEGAL DETAILS, REFER TO THE FULL LICENSE AGREEMENT.
  
  ------------------------------------------------------------------------------------------------
  License: GNU General Public License Version 3 (GPLv3) - https://www.gnu.org/licenses/gpl.html
  ------------------------------------------------------------------------------------------------
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses/
===================================================================================================
*/

#ifndef eRCaGuy_Peer2Peer_h
#define eRCaGuy_Peer2Peer_h

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Stream.h>

// //for future optimization by differentiating between AVR vs ARM-based boards:
// //-see here for where I copied/pasted the below defines from: https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/Arduino.h 
// //-and here: https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/bootloaders/optiboot/pin_defs.h
// #if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  // #define AVR
// #else 
  // #define ARM 
// #endif

/*
***IMPORTANT NOTE ABOUT RING BUFFERS***
"A 'power of 2' buffer size is recommended to dramatically optimize all the modulo operations for ring buffers." (source: https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/HardwareSerial.h)
-previous testing in a different Arduino project shows this to be absolutely true! It appears that using a ring buffer length that is a power of 2 (ex: 2, 4, 8, 16, 32, 64, etc) is about 2x or more faster than other ring buffer sizes near that size. ~GS 
*/
#define _PEER2PEER_TX_BUFF_SIZE 64 //Tx buffer length, make power of 2 for ring buffer modulus operator speed 
#define _PEER2PEER_RX_BUFF_SIZE 64 //Rx buffer length, make power of 2 for ring buffer modulus operator speed

//struct for sendReceive return values 
struct sendReceive_t 
{
  unsigned int bytesSent = 0;
  unsigned int bytesReceived = 0;
  bool timedOut = false;
};

class eRCaGuy_Peer2Peer : public Stream //note that Stream inherits Print 
{  
  public:
    eRCaGuy_Peer2Peer(byte RxPin, byte TxPin, unsigned int timeout_ms=1000, unsigned int clockDelay_us=200); //class constructor; <--TODO: FIND AN OPTIMAL/MORE APPROPRIATE DEFAULT VALUE FOR clockDelay_us ONCE I GET IT ALL WORKING 
    ~eRCaGuy_Peer2Peer(); //destructor 
    
    void begin();
    void end();
    void setTimeout(unsigned int timeout_ms);
    void setClockDelay(unsigned int clockDelay_us);
    sendReceive_t sendReceive(); //send and receive data; call as frequently as possible to minimize blocking time delay the sender must sit and wait for the receiver to talk 
    
    //public virtual Print methods to implement 
    virtual size_t write(uint8_t myByte); 
    // virtual size_t write(const uint8_t *buffer, size_t size);
    virtual int availableForWrite();
    //public virtual Stream methods to implement
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush(); 
    
    // using Print::write; //as SoftwareSerial.h does too, this line gives us access to the Print class's overloaded, *non*virtual implementations of write, while still defining the write method above as the default one to use when calling write through this Peer2Peer class, due to polymorphism. Some of the overloaded implementations of write defined in Print allow printing entire buffers (arrays of bytes), for instance. 
    //See here for an Arduino demonstration usage of the above: https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/HardwareSerial.h
    //-it says "using Print::write; // pull in write(str) and write(buf, size) from Print"
    //Also see here: https://arduino.stackexchange.com/questions/38965/help-understanding-printwrite-it-calls-write-where-is-write-defined-h 
  
  private:
    unsigned int _timeout_ms; //ms; the timeout value until the sendReceive() function gives up when waiting for the receiver to respond  
    unsigned int _clockDelay_us; //us; the forced delay the receiver will wait after clocking a new edge (in order to give the sender time to set its new data bit state) and before the receiver reads this new data bit and clocks yet another edge 
    
    //for fastDigitalRead command
    
    //for fastDigitalWrite command 
    
    //pins
    byte _TxPin; 
    byte _RxPin;
    
    //for ring buffers:
    byte _TxBuff[_PEER2PEER_TX_BUFF_SIZE];
    byte _RxBuff[_PEER2PEER_RX_BUFF_SIZE];
    
    unsigned int _TxBuffReadLoc;
    unsigned int _TxBuffWriteLoc;
    unsigned int _TxBuffNumBytes; 
    
    unsigned int _RxBuffReadLoc;
    unsigned int _RxBuffWriteLoc; 
    unsigned int _RxBuffNumBytes;
    
    //private methods
    bool placeByteInTxBuff(byte byteOut);
    unsigned int receiveData();
    unsigned int sendData();
    unsigned int receiveByte();
    unsigned int sendByte(byte byteToSend, bool isFirstByteToSend=false);
  
};

#endif


