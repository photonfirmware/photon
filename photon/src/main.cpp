/*
Photon Feeder Firmware
Part of the LumenPnP Project
MPL v2
2025
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

// Flags
bool drive_mode = false;
bool driving = false;
bool driving_direction = false;

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

//-------
//SETUP
//-------

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  feeder->set_rgb(false, false, false);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);

  // Setup Feeder
  feeder = new PhotonFeeder(DRIVE1, DRIVE2, PEEL1, PEEL2, LED_R, LED_G, LED_B, &encoder);
  network = new PhotonNetworkLayer(&bus, &packetizer, &addressFilter, &feederFloor);
  protocol = new PhotonFeederProtocol(feeder, &feederFloor, network, UniqueID, UniqueIDsize);

  byte addr = feederFloor.read_floor_address();

  if(addr == 0xFF){ // not detected, turn red
    feeder->set_rgb(true, false, false);
  }
  else if (addr == 0x00){ //not programmed, turn blue
    feeder->set_rgb(false, false, true);
  }

  //Starting rs-485 serial
  ser.begin(BAUD_RATE);

  // attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), checkPosition, CHANGE);

  feeder->resetEncoderPosition(0);
  feeder->setMmPosition(0);

}

void lifetime(){
  // lifetime testing loop
  uint32_t counter = millis();
  uint32_t interval = 3000;
  while(true){
    if(millis() > counter + interval){
      //reset counter to millis()
      counter = millis();
      //move
      feeder->feedDistance(40, true);
      feeder->resetEncoderPosition(0);
      feeder->setMmPosition(0);
    }
  }
}

void showVersion(){

  feeder->showVersion();

}

void topShortPress(){
  //turn led green for movement
  feeder->set_rgb(true, true, true);
  // move forward 4mm
  feeder->feedDistance(20, true);

  if (feeder->getMoveResult() == PhotonFeeder::FeedResult::SUCCESS){
    feeder->set_rgb(false, false, false);
  }
  else{
    feeder->set_rgb(true, false, false);
  }
}

void bottomShortPress(){
  //turn led green for movement
  feeder->set_rgb(true, true, true);
  // move forward 2mm
  feeder->feedDistance(20, false);

  if (feeder->getMoveResult() == PhotonFeeder::FeedResult::SUCCESS){
    feeder->set_rgb(false, false, false);
  }
  else{
    feeder->set_rgb(true, false, false);
  }
}

void topLongPress(){
  //we've got a long top press, lets drive forward, tape or film depending on drive_mode
  if(drive_mode){
    feeder->peel(true);
  }
  else{
    //resetting first feed, since we could now have a new tape type
    feeder->_first_feed_since_load = true;
    feeder->drive(true);
  }
      // set flag for concurrency to know driving state
  driving = true;
  driving_direction = true;
}

void bottomLongPress(){
  // moving in reverse, motor selected by drive_mode
  if(drive_mode){
    feeder->peel(false);
  }
  else{
    //resetting first feed, since we could now have a new tape type
    feeder->_first_feed_since_load = true;
    feeder->drive(false);
  }
    // set flag for concurrency to know driving state
  driving = true;
  driving_direction = false;

}

void bothLongPress(){
  //both are pressed, switching if we are driving tape or film
  
  if(drive_mode){
    feeder->set_rgb(false, false, true);
    drive_mode = false;
  }
  else{
    feeder->set_rgb(true, true, false);
    drive_mode = true;
  }

  //if both are held for a long time, we show current version id
  uint32_t timerStart = millis();

  bool alreadyFlashed = false;

  while( (!digitalRead(SW1) || !digitalRead(SW2))){
    //do nothing while waiting for debounce
    if((timerStart + 2000 < millis()) && !alreadyFlashed){
      feeder->set_rgb(false, false, false);
      showVersion(); 
      alreadyFlashed = true;
    }
  }

  //delay for debounce
  delay(50);
  feeder->set_rgb(false, false, false);
}

inline void checkButtons() {
  if(!driving){
    // Checking bottom button
    if(!digitalRead(SW1)){
      delay(LONG_PRESS_DELAY);
      // if bottom long press
      if(!digitalRead(SW1)){
        // if both long press
        if(!digitalRead(SW2)){
          bothLongPress();
        }
        // if just bottom long press
        else{
          bottomLongPress();
        }
      }
      // if bottom short press
      else{
        bottomShortPress(); 
      }
    }
    // Checking top button
    if(!digitalRead(SW2)){
      delay(LONG_PRESS_DELAY);
      // if top long press
      if(!digitalRead(SW2)){
        // if both long press
        if(!digitalRead(SW1)){
          bothLongPress();
        }
        // if just top long press
        else{
          topLongPress();
        }
      }
      // if top short press
      else{
        topShortPress();
      }  
    }
  }
  else{
    if((driving_direction && digitalRead(SW2)) || (!driving_direction && digitalRead(SW1))){
      //stop all motors
      feeder->halt();
      //reset encoder and mm position
      feeder->resetEncoderPosition(0);
      feeder->setMmPosition(0);
      driving = false;
      delay(5);
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
  checkButtons();
  checkForRS485Packet();
}
