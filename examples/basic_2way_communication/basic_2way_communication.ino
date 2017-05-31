/*
basic_2way_communication 
 - get 2 Arduinos to talk back and forth to each other
By Gabriel Staples
http://www.ElectricRCAircraftGuy.com 
My contact info is available by clicking the "Contact Me" tab at the top of my website.
Written: 30 May 2017 
Updated: 30 May 2017  

LICENSE: GNU GPLV3 or later (refer to .h file and attached license for details)

INSTRUCTIONS:
 - load this code onto 2 Arduinos 

CIRCUIT:
 - connect Arduino1 A0 to Arduino2 A1, preferably via a 1k~10k resistor to protect the pins in case you plug it in wrong 
 - connect Arduino1 A1 to Arduino2 A0, preferably via a 1k~10k resistor to protect the pins in case you plug it in wrong 

*/

#include <eRCaGuy_Peer2Peer.h>

//pins (choose any two pins)
const byte RX_PIN = A0;
const byte TX_PIN = A1;

eRCaGuy_Peer2Peer peer1(RX_PIN, TX_PIN);

//-----------------------------------------------------------------------------------------------
//setup 
//-runs once at power-up 
//-----------------------------------------------------------------------------------------------
void setup()
{
  //Open serial communications and wait for port to open
  Serial.begin(115200);
  while (!Serial) {} //wait for serial port to connect. Needed for native USB ports only (ex: on 32u4 mcu).
  Serial.println("Begin");
  
  peer1.begin();
  peer1.println("From peer1.");
}

//-----------------------------------------------------------------------------------------------
//loop 
//-runs over and over again 
//-----------------------------------------------------------------------------------------------
void loop() 
{ 
  if (peer1.available()) //if bytes are ready for reading 
  {
    Serial.write(peer1.read());
  }
  
  if (Serial.available()) 
  {
    peer1.write(Serial.read());
  }
}





