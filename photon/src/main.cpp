/*
Photon Feeder Firmware
Part of the LumenPnP Project
GNU GPL v3
2022
*/

#include "define.h"

#ifdef UNIT_TEST
  #include <ArduinoFake.h>
#else
  #include <Arduino.h>
  #include <HardwareSerial.h>
  #include <OneWire.h>
  #include <ArduinoUniqueID.h>
  #include <rs485/rs485bus.hpp>
#endif // UNIT_TEST

#ifndef MOTOR_DEPS
#define MOTOR_DEPS

#include <RotaryEncoder.h>

#endif 

#include "FeederFloor.h"
#include "PhotonFeeder.h"
#include "PhotonFeederProtocol.h"
#include "PhotonNetworkLayer.h"

#include <rs485/rs485bus.hpp>
#include <rs485/bus_adapters/hardware_serial.h>
#include <rs485/filters/filter_by_value.h>
#include <rs485/protocols/photon.h>
#include <rs485/packetizer.h>

#define BAUD_RATE 57600

//-----
// Global Variables
//-----

#ifdef UNIT_TEST
StreamFake ser();
#else
HardwareSerial ser(PA10, PA9);
#endif // ARDUINO

// EEPROM
OneWire oneWire(ONE_WIRE);
FeederFloor feederFloor(&oneWire);

// RS485
HardwareSerialBusIO busIO(&ser);
RS485Bus<RS485_BUS_BUFFER_SIZE> bus(busIO, _RE, DE);
PhotonProtocol photon_protocol;
Packetizer packetizer(bus, photon_protocol);
FilterByValue addressFilter(0);

// Encoder
RotaryEncoder encoder(DRIVE_ENC_A, DRIVE_ENC_B, RotaryEncoder::LatchMode::TWO03); 

// PID
double Setpoint, Input, Output;

// Drive mode toggle
bool drive_mode = false;

// Feeder Class Instances
PhotonFeeder *feeder;
PhotonFeederProtocol *protocol;
PhotonNetworkLayer *network;

//-------
//FUNCTIONS
//-------

void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

void bootAnimation(){
  for(int i = 0;i<3;i++){
    feeder->set_rgb(true, true, true);

    delay(100);

    feeder->set_rgb(false, false, false);

    delay(100);
  }

}

//-------
//SETUP
//-------

void setup() {
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  // Setup Feeder
  feeder = new PhotonFeeder(DRIVE1, DRIVE2, PEEL1, PEEL2, LED_R, LED_G, LED_B, &encoder);
  network = new PhotonNetworkLayer(&bus, &packetizer, &addressFilter, &feederFloor);
  protocol = new PhotonFeederProtocol(feeder, &feederFloor, network, UniqueID, UniqueIDsize);
  
  bootAnimation();

  // put current floor address on the leds if detected

  // addr = read_floor_address();

  // if(addr == 0xFF){ // not detected, turn red
  //   feeder->set_rgb(true, false, false);
  // }
  // else if (addr == 0x00){ //not programmed, turn blue
  //   feeder->set_rgb(false, false, true);
  // }

  //Starting rs-485 serial
  ser.begin(BAUD_RATE);

  // attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), checkPosition, CHANGE);

  feeder->checkLoaded();
}

void lifetime(){
  // lifetime testing loop
  unsigned long counter = millis();
  int interval = 3000;
  while(true){
    if(millis() > counter + interval){
      //reset counter to millis()
      counter = millis();
      //move
      feeder->feedDistance(40, true);
      feeder->setEncoderPosition(0);
      feeder->setMmPosition(0);
    }
  }
}

inline void keyPressedOne() {
  // Checking SW1 status to go backward, or initiate settings mode
  if(!digitalRead(SW1)){
    delay(LONG_PRESS_DELAY);

    if(!digitalRead(SW1)){

      if(!digitalRead(SW2)){
        //both are pressed, entering settings mode
        feeder->set_rgb(false, false, true);
        if(drive_mode){
          drive_mode = false;
        }
        else{
          drive_mode = true;
        }
        while(!digitalRead(SW1) || !digitalRead(SW2)){
          //do nothing while waiting for debounce
        }
        feeder->set_rgb(false, false, false);
      }
      else{
        //we've got a long press, lets reverse 
        while(!digitalRead(SW1)){

          if(drive_mode){
            feeder->peel(50, false);
          }
          else{
            feeder->driveTape(false);
          }
        }
        feeder->brakeDrive(200);
        feeder->stop();
        feeder->setEncoderPosition(0);
        feeder->setMmPosition(0);
        
      }
      
    }
    else{
      //feeder->feedDistance(40, false);
      //feeder->checkLoaded();
      lifetime();
      
    }
  }
}

inline void keyPressedTwo() {
  // Checking SW2 status to go forward
  if(!digitalRead(SW2)){
    delay(LONG_PRESS_DELAY);

    if(!digitalRead(SW2)){

      if(!digitalRead(SW1)){
        //both are pressed, entering settings mode
        feeder->set_rgb(false, false, true);
        if(drive_mode){
          drive_mode = false;
        }
        else{
          drive_mode = true;
        }
        while(!digitalRead(SW1) || !digitalRead(SW2)){
          //do nothing while waiting for debounce
        }
        feeder->set_rgb(false, false, false);
      }
      else{
        //we've got a long press, lets peel film 
        while(!digitalRead(SW2)){
          if(drive_mode){
            feeder->peel(50, true);
          }
          else{feeder->driveTape(true);}
        }
        feeder->brakeDrive(200);
        feeder->stop();
        feeder->setEncoderPosition(0);
        feeder->setMmPosition(0);
      }
      
    }
    else{
      feeder->set_rgb(false, true, false);
      if(feeder->feedDistance(40, true) == PhotonFeeder::FeedResult::SUCCESS){
        feeder->set_rgb(false, false, false);
      }
      else{
        feeder->set_rgb(true, false, false);
      }
      
    }  
  }
}

inline void checkForRS485Packet() {
  protocol->tick();
}

//------
//  MAIN CONTROL LOOP
//------
void loop() {
  keyPressedOne();
  keyPressedTwo();
  checkForRS485Packet();
}
