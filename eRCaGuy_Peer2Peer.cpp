/*
eRCaGuy_Peer2Peer

AUTHOR:
By Gabriel Staples
Website: http://www.ElectricRCAircraftGuy.com
My contact info is available by clicking the "Contact Me" tab at the top of my website.

FOR PERTINENT INFORMATION ABOUT THIS LIBRARY SEE THE TOP OF THE HEADER (.h) FILE, AND THE COMMENTS IN BOTH THIS FILE AND THE HEADER FILE.
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

/*
References: 
 - 
 - 


 
*/

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "eRCaGuy_Peer2Peer.h"

//macros
#define TIMED_OUT (65535)

//=================================================================================================
//Class Constructor & Destructor
//-note they have no return types, whereas all other methods do 
//=================================================================================================
//Constructor 
eRCaGuy_Peer2Peer::eRCaGuy_Peer2Peer(byte RxPin, byte TxPin, unsigned int timeout_ms, unsigned int clockDelay_us)
{
  _TxPin = TxPin; 
  _RxPin = RxPin; 
  
  _TxBuffReadLoc = 0;
  _TxBuffWriteLoc = 0;
  _TxBuffNumBytes = 0;
  
  _RxBuffReadLoc = 0;
  _RxBuffWriteLoc = 0;
  _RxBuffNumBytes = 0;
  
  _timeout_ms = timeout_ms; //ms
  _clockDelay_us = clockDelay_us; //us 
}

//Destructor 
eRCaGuy_Peer2Peer::~eRCaGuy_Peer2Peer()
{
  end();
}

//=================================================================================================
//Other class methods
//=================================================================================================

//-------------------------------------------------------------------------------------------------
//begin 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::begin()
{
  pinMode(_TxPin, OUTPUT);
  pinMode(_RxPin, INPUT);
  
  //start out with Tx pin LOW
  digitalWrite(_TxPin, LOW);
}

//-------------------------------------------------------------------------------------------------
//end
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::end()
{
  //set both to inputs to protect the pins by making them high impedance 
  pinMode(_TxPin, INPUT);
  pinMode(_RxPin, INPUT);
}

//-------------------------------------------------------------------------------------------------
//setTimeout
//-set the timeout, which is essentially how long we will wait in sendReceive when waiting for a receiver to respond to a sender wanting to send 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::setTimeout(unsigned int timeout_ms)
{
  _timeout_ms = timeout_ms; //ms 
}

//-------------------------------------------------------------------------------------------------
//setClockDelay
//-set the clockDelay, which is the forced delay the receiver will wait after clocking a new edge (in order to give the sender time to set its new data bit state) and before the receiver reads this new data bit and clocks yet another edge 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::setClockDelay(unsigned int clockDelay_us)
{
  _clockDelay_us = clockDelay_us; //us 
}

//-------------------------------------------------------------------------------------------------
//sendReceive
//-do the actual sending and receiving here
//-NB: this is a BLOCKING command! ie: if there is data to be sent, it will BLOCK until either the receiver responds and accepts the data, or it times out; if there is data to be received, it will block while receiving it 
//-call as frequently as possible to minimize the blocking time delay that other senders must sit and wait for this device to receive and talk 
//-returns a struct of type sendReceive_t, defined in the header file 
//-------------------------------------------------------------------------------------------------
/*
Description of send/receive protocol:
 - Tx = "transmitter" or "sender" below 
 - Rx = "receiver" below 

Tx writes pin HIGH to say "I want to send data". Rx comes around and eventually sees that and writes pin HIGH to say "I acknowledge you want to send." Once the Tx sees this, it disables interrupts and writes Tx pin LOW to say, "start clocking data for me." Rx disables interrupts and starts clocking bits by writing rising and falling edges (pin toggling). After each clock edge it writes it waits a FIXED_TIME_PD called "_clockDelay_us" in order to give the Tx time to set its data state on its Tx line. Since *both* the Tx and Rx have their interrupts *disabled* during all bit-sends, we can guarantee that A) the sender is always listening and responding within a fixed, determinate amount of time, and B) the time that interrupts are disabled is minimized because the receiver can NOT be interrupted either (thereby also prolonging the Tx's disabled-interrupt time) while clocking bits since its interrupts are ALSO disabled. 

After the receiver (Rx) clocks 8 bits by toggling its Tx pin state 8 times and waiting the appropriate _clockDelay_us time pd after each clock edge and before reading the Tx's data state, it stops, writes its Tx pin LOW(*), and *enables* interrupts. Once the Tx has seen 8 clocked edges from the Rx it stops too, and it *enables* interrupts. Then, the Tx writes its Tx pin state HIGH again to say, "I want to send the next byte." At this moment, the Tx ensures _clockDelay_us time has passed BEFORE reading the acknowledgement pin from the Rx, to give the Rx adequate time to set its acknowledgement line LOW (see (*) point just above) before enabling and processing any interrupts (otherwise the Tx may read a false-positive acknowledgement from the Rx when the Rx is simply stuck processing a long ISR). The Tx's interrupts are still *en*abled. The Rx (interrupts still *en*abled), sees this HIGH state, and writes its Tx pin HIGH to say "I acknowledge you want to send." The sequence begins again as described above. 

Q: What if the Tx has a really long interrupt (ISR) right after it *enables* interrupts but before it writes its Tx pin state HIGH again to say, "I want to send the next byte."?
A: Everything is ok. The Rx does NOT go on without it, thinking it has no more data to send, since it knows that it must receive *at least* 2 bytes for the _TxBuffNumBytes variable, + that number of bytes, before the sender is done sending. Therefore, it will simply sit around until either the sender is ready again or it times out, then the sending will continue. 
Q: What if the *Tx* has a really long interrupt after it writes its pin HIGH and before noticing that the Rx acknowledges? Will this mess up the sequence?
A: NO, because once the Rx acknowledges, if the *Tx* is still stuck in a long ISR, it will simply take longer to exit the ISR, detect that the Rx acknowledged (the Rx will simply be sitting there blocking and waiting), disable interrupts, and write its Tx pin LOW to say, "start clocking data for me." No problems caused, other than making the Rx block a little longer while the Tx finished its ISR so it could tell the Rx to start clocking data for it. 
Q: What if the *Rx* has a really long interrupt after clocking all 8 bits, waiting the appropriate _clockDelay_us time, reading the 8th bit from the Tx, writing its Tx pin LOW, and enabling interrupts? Will this mess up the sequence?
A: NO, because this is the same as the Tx initially simply sitting around in a blocking wait waiting for the Rx to notice that it wants to send. Once the Rx exits its ISR, it will notice the Tx wants to send, write its pin HIGH to say "I acknowledge you want to send," and wait for the Tx to write its pin LOW to say, "start clocking data for me." No problems caused, other than making the Tx block a little longer while the Rx finished its ISR so it could acknowledge that the Tx wants to send. 
*/
sendReceive_t eRCaGuy_Peer2Peer::sendReceive()
{
  sendReceive_t sendReceiveState; //create a struct and load it with default values, as defined in the struct definition 
  
  //1st, RECEIVE DATA
  unsigned int receiveState = receiveData();
  if (receiveState==TIMED_OUT)
    sendReceiveState.timedOut = true;
  else
    sendReceiveState.bytesReceived = receiveState;
  
  //2nd, SEND DATA, if we have any to send 
  unsigned int sendState = sendData();
  if (sendState==TIMED_OUT)
    sendReceiveState.timedOut = true;
  else 
    sendReceiveState.bytesSent = sendState;
  
  //3rd, RECEIVE DATA again! 
  //-Otherwise, you might waste a loop cycle before you receive the other one's data even though he wanted to send the whole time! It's just that he was super polite and received your data first (notice above--we receive first, BEFORE trying to send). So, be a gentleman and receive his data now before you go running off again!
  receiveState = receiveData();
  if (receiveState==TIMED_OUT)
    sendReceiveState.timedOut = true;
  else
    sendReceiveState.bytesReceived += receiveState;
  
  return sendReceiveState;
}

//-------------------------------------------------------------------------------------------------
//receiveData
//-private method 
//-returns # data bytes received (NOT counting the first 2 bytes which specify how many bytes are going to be sent), or 65535 ("TIMED_OUT") if it times out 
//-------------------------------------------------------------------------------------------------
unsigned int eRCaGuy_Peer2Peer::receiveData()
{
  unsigned int bytesReceived = 0;
  
  //-check to see if Rx pin is HIGH, which indicates the other device wants to SEND data 
  if (digitalRead(_RxPin)==HIGH)
  {
    //prepare to read in the first 2 bytes as the # indicating how many bytes will be sent 
    byte lowerByte, upperByte;
    lowerByte = receiveByte();
    if (lowerByte!=TIMED_OUT)
    {
      upperByte = receiveByte();
      if (upperByte!=TIMED_OUT)
      {
        unsigned int bytesToReceive = (upperByte<<8) | lowerByte;
        //receive all incoming bytes, unless timeout occurs 
        for (unsigned int i=0; i<bytesToReceive; i++)
        {
          byte byteIn = receiveByte();
          if (byteIn==TIMED_OUT)
            break;
          else if (_RxBuffNumBytes<_PEER2PEER_RX_BUFF_SIZE)
          {
            bytesReceived++;
            //only store the byteIn if there is space in the Rx buffer, otherwise just throw the byte away
            _RxBuff[_RxBuffWriteLoc] = byteIn;
            _RxBuffNumBytes++;
            _RxBuffWriteLoc = (_RxBuffWriteLoc + 1) % _PEER2PEER_RX_BUFF_SIZE; //increment & wrap 
          }
        }
      }
    }
  }
  
  return bytesReceived;
}

//-------------------------------------------------------------------------------------------------
//sendData
//-private method 
//-returns # bytes sent (NOT counting the first 2 bytes which specify the # of data bytes to be sent), or -1 (same as 65535 actually, or "TIMED_OUT", since it is an unsigned value) if it times out 
//-------------------------------------------------------------------------------------------------
unsigned int eRCaGuy_Peer2Peer::sendData()
{
  unsigned int bytesSent = 0; ////////TODO: handle bytesSent variable, and the timeout code///////
  
  if (_TxBuffNumBytes>0)
  {
    //There is data to be sent, so send it 
    //prepare _TxBuffNumBytes variable to send, then send it Least Significant Byte first 
    byte lowerByte = _TxBuffNumBytes&0xFF;
    byte upperByte = (_TxBuffNumBytes>>8)&0xFF;
    sendByte(lowerByte, true);
    sendByte(upperByte);
    //clock out the data 1 byte at a time 
    while (_TxBuffNumBytes>0)
    {
      byte byteToSend = _TxBuff[_TxBuffReadLoc];
      _TxBuffReadLoc = (_TxBuffReadLoc + 1) % _PEER2PEER_TX_BUFF_SIZE; //increment & wrap 
      _TxBuffNumBytes--;
      sendByte(byteToSend);
    }
  }
  
  return bytesSent; //////////////
}

//-------------------------------------------------------------------------------------------------
//receiveByte
//-private method 
//-don't ever call this receiveByte() function unless we KNOW another byte is supposed to be coming in
//-returns the byte value if a byte was successfully received, or 65535 if it timed out 
//-------------------------------------------------------------------------------------------------
unsigned int eRCaGuy_Peer2Peer::receiveByte()
{
  byte byteIn;
  
  //we don't ever call this receiveByte() function unless we KNOW another byte is supposed to be coming in
  //-therefore, do a blocking wait until the _RxPin goes HIGH, indicating it is ready to send data again (ex: after processing interrupts between sending bytes)
  while (digitalRead(_RxPin)==LOW){} //wait for _RxPin to go HIGH 

  //There is data to be received, so receive it
  //1st, write _TxPin HIGH to say, "I acknowledge you want to send."
  bool TxPinState = HIGH;
  digitalWrite(_TxPin, TxPinState);
  //2nd, wait until sender writes Tx pin LOW to tell receiver "start clocking data for me"
  while (digitalRead(_RxPin)==HIGH){}
  //sender is ready to receive data, so start clocking bits
  //-disable interrupts first to minimize time the sender has its interrupts disabled (ie: don't let the receiver's interrupt routines slow down this process either, so disable interrupts on the receiver side here as well)
  byte SREG_bak = SREG; //back up interrupt state
  noInterrupts();
  //now clock 8 bits 
  for (byte i=0; i<8; i++)
  {
    //toggle pin and write it 
    TxPinState = !TxPinState; //toggle; //////////////TODO: FUTURE WORK--FOR AVR MCUS, REPLACE THIS WITH A SINGLE XOR COMMAND (^) TO TOGGLE THE PIN DIRECTLY AT THE REGISTER LEVEL IN A SINGLE CLOCK INSTRUCTION
    digitalWrite(_TxPin, TxPinState);
    //wait a FIXED_TIME_PD called "_clockDelay_us" in order to give the Tx time to set its data state on its Tx line
    delayMicroseconds(_clockDelay_us);
    //now read the data bit from the sender before clocking another bit 
    bitWrite(byteIn, i, digitalRead(_RxPin));
  }
  digitalWrite(_TxPin, LOW); //prepare for next byte 
  SREG = SREG_bak; //restore interrupt state 
  
  return byteIn; //////////////
}

//-------------------------------------------------------------------------------------------------
//sendByte
//-private method 
//-returns 1 if a byte was successfully sent, or 65535 if it timed out 
//-------------------------------------------------------------------------------------------------
unsigned int eRCaGuy_Peer2Peer::sendByte(byte byteToSend, bool isFirstByteToSend)
{
  //1st, set Tx pin HIGH to tell the other device, "I want to send"
  digitalWrite(_TxPin, HIGH);
  //2nd, delay for all but the first byte to be sent 
  if (!isFirstByteToSend)
    delayMicroseconds(_clockDelay_us); //no need to do this for the first byte only, so avoid it for the first byte to speed things up a tiny bit 
  //3rd, poll and wait until the Rx pin goes HIGH which means the other device is ready to receive!
  while (digitalRead(_RxPin==LOW)){}
  
  //save RxPinState_old so we can detect clock edges (pin state toggles) 
  bool RxPinState_old = HIGH; //this was the last known state before getting to this point 
  //ok, _RxPin is HIGH now, so disable interrupts and write Tx pin LOW to say, "start clocking data for me"
  byte SREG_bak = SREG; //back up interrupt state
  noInterrupts();
  digitalWrite(_TxPin, LOW); 
  for (byte i=0; i<8; i++)
  {
    while (digitalRead(_RxPin)==RxPinState_old){} //poll until Rx pin (clock signal) changes 
    RxPinState_old = !RxPinState_old; //update (pin state just toggled, so just toggle the variable) 
    digitalWrite(_TxPin, bitRead(byteToSend, i)); //write next data bit 
  }
  SREG = SREG_bak; //restore interrupt state 
  
  return 1; ///////////////////////TODO: FIX THE RETURN 
}

//-------------------------------------------------------------------------------------------------
//write
//-virtual method 
//-place the byte into the write buffer, or if the write buffer is full, do a blocking sendReceive and then try again to place it in  
//-------------------------------------------------------------------------------------------------
size_t eRCaGuy_Peer2Peer::write(uint8_t byteOut)
{  
  //store byteOut into the Tx buffer so long as it's not full 
  size_t bytesLoaded = placeByteInTxBuff(byteOut);
  
  if (bytesLoaded==0)
  {
    //_TxBuff is full, so do a blocking sendReceive and then try again
    sendReceive();
    bytesLoaded = placeByteInTxBuff(byteOut); //try again, this time giving up if still no luck, as it means the receiver isn't responding and sendReceive is timing out 
  }
  
  return bytesLoaded;
}

//-------------------------------------------------------------------------------------------------
//placeByteInTxBuff
//-private method 
//-place the byte into the Tx buffer
//-returns the number of bytes placed into the Tx buffer (0 or 1)
//--therefore, if it returns 0, it means the Tx buffer is currently full
//-------------------------------------------------------------------------------------------------
bool eRCaGuy_Peer2Peer::placeByteInTxBuff(byte byteOut)
{
  //store byteOut into the Tx buffer so long as it's not full 
  bool bytesLoaded = 0;
  if (_TxBuffNumBytes<_PEER2PEER_TX_BUFF_SIZE)
  {
    _TxBuff[_TxBuffWriteLoc] = byteOut;
    _TxBuffNumBytes++;
    _TxBuffWriteLoc = (_TxBuffWriteLoc + 1) % _PEER2PEER_TX_BUFF_SIZE; //increment writeLoc 
    bytesLoaded = 1;
  }
  return bytesLoaded;
}

//-------------------------------------------------------------------------------------------------
//availableForWrite
//-virtual method 
//-returns # bytes sitting in the OUTGOING (Tx) buffer 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::availableForWrite()
{
  return (int)_TxBuffNumBytes;
}

//-------------------------------------------------------------------------------------------------
//available
//-virtual method 
//-returns # bytes sitting in the INCOMING (Rx) buffer 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::available()
{
  return (int)_RxBuffNumBytes;
}

//-------------------------------------------------------------------------------------------------
//read
//-virtual method 
//-implements the Stream read method here: https://www.arduino.cc/en/Reference/StreamRead 
//-returns the first byte of incoming data or -1 if no data is available 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::read()
{
  if (_RxBuffNumBytes==0)
    return -1;
  //else _RxBuffNumBytes>0
  byte byteIn = _RxBuff[_RxBuffReadLoc];
  _RxBuffNumBytes--;
  _RxBuffReadLoc = (_RxBuffReadLoc + 1) % _PEER2PEER_RX_BUFF_SIZE; //increment and wrap around
  return byteIn;
}

//-------------------------------------------------------------------------------------------------
//peek
//-virtual method 
//-implements the Stream peek method here: https://www.arduino.cc/en/Reference/StreamPeek
//-Read a byte from the file without advancing to the next one. That is, successive calls to peek() will return the same value, as will the next call to read().
//-returns the next byte (or character), or -1 if none is available.
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::peek()
{
  if (_RxBuffNumBytes==0)
    return -1;
  //else _RxBuffNumBytes>0
  return _RxBuff[_RxBuffReadLoc];
}

//-------------------------------------------------------------------------------------------------
//flush 
//-virtual method 
//-implements the Stream flush method here: https://www.arduino.cc/en/Reference/StreamFlush
//-is basically a blocking write, forcing the Tx buffer to be allowed to empty before returning, unless it times out 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::flush()
{
  sendReceive();
}






