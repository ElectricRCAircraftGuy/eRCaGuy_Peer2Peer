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
sendReceive_t eRCaGuy_Peer2Peer::sendReceive()
{
  ///////////TODO (IN WORK)///////////////
  
  sendReceive_t sendReceiveState; //create a struct and load it with default values, as defined in the struct definition 
  
  
  
  return sendReceiveState;
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






