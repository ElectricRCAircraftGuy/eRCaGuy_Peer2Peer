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

//Macros 
//COMMENT OUT THE STATEMENT BELOW THAT IS NOT TRUE
//-UPLOAD THE ONE TO ONE ARDUINO, AND THE OTHER TO ANOTHER 
// #define I_AM_ARDUINO_1 //comment out if not true 
#define I_AM_ARDUINO_2 //comment out if not true 

#if defined(I_AM_ARDUINO_1)
  const char MY_NAME[] = "Arduino 1";
#elif defined(I_AM_ARDUINO_2)
  const char MY_NAME[] = "Arduino 2";
#endif 

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
  Serial.print("Begin. This is "); Serial.println(MY_NAME);
  
  //send data from one peer to another to start 
  peer1.begin();
  peer1.print("From "); peer1.println(MY_NAME);
}

//-----------------------------------------------------------------------------------------------
//loop 
//-runs over and over again 
//-----------------------------------------------------------------------------------------------
void loop() 
{ 
  //if bytes are ready for reading from another peer (ex: Arduino)
  if (peer1.available()) 
  {
    //write to the Serial Monitor 
    Serial.write(peer1.read());
  }
  
  //if bytes are ready for reading from the user's Serial Monitor 
  if (Serial.available()) 
  {
    char cIn = Serial.read();
    
    Serial.write(cIn); //DEBUG TEST 
    //write to the peer (ex: another Arduino) 
    peer1.write(cIn);
  }
  
  peer1.sendReceive(); //send and receive data; call as frequently as possible to minimize blocking time delay the sender must sit and wait for the receiver to talk 
  
  // Serial.println("test"); //DEBUGGING 
  
  //DEBUGGING
  pinMode(13, OUTPUT);
  static bool LEDState = HIGH;
  LEDState = !LEDState;
  digitalWrite(13, LEDState);
  
  // //DEBUG TEST
  // while (true)
  // {
    // pinMode(13, OUTPUT);
    // digitalWrite(13, HIGH);
    // delay(100);
    // digitalWrite(13, LOW);
    // delay(100);
  // }
}





