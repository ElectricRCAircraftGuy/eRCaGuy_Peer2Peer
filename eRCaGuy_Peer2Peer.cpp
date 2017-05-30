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
eRCaGuy_Peer2Peer::eRCaGuy_Peer2Peer(byte RxPin, byte TxPin)
{
  _TxPin = TxPin; 
  _RxPin = RxPin; 
  
  _TxBuffReadLoc = 0;
  _TxBuffWriteLoc = 0;
  _TxBuffNumBytes = 0;
  
  _RxBuffReadLoc = 0;
  _RxBuffWriteLoc = 0;
  _RxBuffNumBytes = 0;
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
//sendReceive
//-do the actual sending and receiving here
//-NB: this is a BLOCKING command! ie: if there is data to be sent, it will BLOCK until either the receiver responds and accepts the data, or it times out; if there is data to be received, it will block while receiving it 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::sendReceive()
{

  
}

//-------------------------------------------------------------------------------------------------
//write
//-------------------------------------------------------------------------------------------------
size_t eRCaGuy_Peer2Peer::write(uint8_t myByte)
{  
  //store myByte into the Tx buffer so long as it's not full 
  size_t bytesLoaded = 0;
  if (_TxBuffNumBytes<_PEER2PEER_TX_BUFF_SIZE)
  {
    _TxBuff[_TxBuffWriteLoc] = myByte;
    _TxBuffWriteLoc = (_TxBuffWriteLoc + 1) % _PEER2PEER_TX_BUFF_SIZE; //increment writeLoc 
    bytesLoaded = 1;
  }
  return bytesLoaded;
}

//-------------------------------------------------------------------------------------------------
//availableForWrite
//-returns # bytes sitting in the OUTGOING (Tx) buffer 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::availableForWrite()
{
  
}

//-------------------------------------------------------------------------------------------------
//available
//-returns # bytes sitting in the INCOMING (Rx) buffer 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::available()
{
  
}

//-------------------------------------------------------------------------------------------------
//read
//-implements the Stream read method here: https://www.arduino.cc/en/Reference/StreamRead 
//-returns the first byte of incoming data or -1 if no data is available 
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::read()
{
  
}

//-------------------------------------------------------------------------------------------------
//peek
//-implements the Stream peek method here: https://www.arduino.cc/en/Reference/StreamPeek
//-Read a byte from the file without advancing to the next one. That is, successive calls to peek() will return the same value, as will the next call to read().
//-returns the next byte (or character), or -1 if none is available.
//-------------------------------------------------------------------------------------------------
int eRCaGuy_Peer2Peer::peek()
{
  
}

//-------------------------------------------------------------------------------------------------
//flush 
//-implements the Stream flush method here: https://www.arduino.cc/en/Reference/StreamFlush
//-is basically a blocking write, forcing the Tx buffer to be allowed to empty before returning 
//-------------------------------------------------------------------------------------------------
void eRCaGuy_Peer2Peer::flush()
{
  
}






