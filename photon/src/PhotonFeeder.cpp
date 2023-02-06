#include <Arduino.h>
#include "PhotonFeeder.h"
#include "define.h"

#define TENTH_MM_PER_PIP 40
#define TIMEOUT_PER_PIP

// drive motor data
// gearbox has ratio of 1:1030
// encoder has 14 ticks per revolution
// one full rotation of the output shaft is 14*1030 = 14420 ticks
// divided by 32 teeth is 450.625 ticks per tooth
// divided by 40 tenths of a mm per tooth (4mm) is 11.265625 ticks per tenth mm

// ok it seems i have some drift with the number calculated above
// adjusting based on our drift (2.5 tenths over the course of 5600 tenths), the new rate should be 11.27065654
#define TICKS_PER_TENTH_MM 11.27065654
#define TENSION_TIME_PER_TENTH_MM 20
#define TIMEOUT_TIME_PER_TENTH_MM 25

#define BACKLASH_COMP_TENTH_MM 10
#define BACKWARD_FILM_SLACK_TIMEOUT 200
#define SS_THRESHOLD 1

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255

// Unit Tests Fail Because This Isn't Defined In ArduinoFake for some reason
#ifndef INPUT_ANALOG
#define INPUT_ANALOG 0x04
#endif

PhotonFeeder::PhotonFeeder(
            uint8_t drive1_pin,
            uint8_t drive2_pin,
            uint8_t peel1_pin,
            uint8_t peel2_pin,
            uint8_t led_red,
            uint8_t led_green,
            uint8_t led_blue,
            RotaryEncoder* encoder,
            OneWire* oneWire
        ) :
    _drive1_pin(drive1_pin),
    _drive2_pin(drive2_pin),
    _peel1_pin(peel1_pin),
    _peel2_pin(peel2_pin),
    _led_red(led_red),
    _led_green(led_green),
    _led_blue(led_blue),
    _position(0),
    _encoder(encoder),
    _oneWire(oneWire) {

    pinMode(_drive1_pin, OUTPUT);
    pinMode(_drive2_pin, OUTPUT);
    pinMode(_peel1_pin, OUTPUT);
    pinMode(_peel2_pin, OUTPUT);

    pinMode(_led_red, OUTPUT);
    pinMode(_led_green, OUTPUT);
    pinMode(_led_blue, OUTPUT);
}

Feeder::FeedResult PhotonFeeder::feedDistance(uint16_t tenths_mm, bool forward) {

    // if (abs(tenths_mm) % TENTH_MM_PER_PIP != 0) {
    //     // The Opulo Photon Feeder has only been tested and calibrated for moves of 4mm (One Pip) so far.
    //     // If any other value is supplied, indicate it is invalid.
    //     return Feeder::FeedResult::INVALID_LENGTH;
    // }

    bool success = (forward) ? moveForward(tenths_mm) : moveBackward(tenths_mm);
    if (!success) {
        return FeedResult::MOTOR_FAULT;
    }


    return Feeder::FeedResult::SUCCESS;
}

void PhotonFeeder::brakeDrive(uint16_t brake_time){
    //brings both drive pins high
    analogWrite(_drive1_pin, 255);
    analogWrite(_drive2_pin, 255);

    delay(brake_time);

    stop();

}

/*  checkLoaded()
    The checkLoaded() function checks to see what's loaded in the feeder, and sets PID components appropriately.
    This gets run on first movement command after boot, after quick move, or after the protocol requests it.

*/
bool PhotonFeeder::checkLoaded() {

    //takes up any backlash slack, ensures any forward movement is tape movement
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 250);
    delay(2);

    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 20);
    delay(500);

    stop();

    // find starting threshold of movement
    int errorThreshold = 3;
    int movedAt = 256;
    signed long startingTick, currentTick;

    startingTick = _encoder->getPosition();

    for(int movementIndex = 20; movementIndex<255; movementIndex=movementIndex + 5){

        analogWrite(_drive1_pin, 0);
        analogWrite(_drive2_pin, movementIndex);

        delay(75);

        currentTick = _encoder->getPosition();

        if(abs(startingTick - currentTick) > errorThreshold){
            movedAt = movementIndex;
            break;
        }
    }

    stop();

    // set pid components based on this

    //Empty should be red
    // 0402 is consistently creen
    // 0603 is between green and blue
    // 0805 is

    // Green
    if(movedAt < 48){               // 0402 tape with almost no resistance
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_B, HIGH);

        // Setting PID
        _Kp=1;
        _Ki=0;
        _Kd=2;

    }

    // Yellow
    else if(movedAt < 180){         // 0603, a bit thicker with a bit of resistance
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_B, HIGH);

        // Setting PID
        _Kp=0.5;
        _Ki=0.01;
        _Kd=0.2;

    }

    // PINK
    else if(movedAt < 256){         // Thicc Boi tape
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, LOW);

        // Setting PID
        _Kp=1;
        _Ki=0.01;
        _Kd=10;

    }

    // RED
    else{                           // Full tilt didn't move it, it's gonna need
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, HIGH);

        // Setting PID
        _Kp=1;
        _Ki=0.1;
        _Kd=75;

    }

    delay(250);

    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);

    return true;

}

/* moveInternal()
*   This function actually handles the translation of the mm movement to driving the motor to the right position based on encoder ticks.
*   We can't just calculate the number of ticks we need to move for the given mm movement requested, and increment our tick count by that much.
*   Because the tick count is only ever an approximation of the precise mm position, any rounding done from the mm->tick conversion will
*   result in a signficiant amount of accrued error.
*
*   Instead, we need to use the _mm_ position as ground truth, and only ever use the ticks as only how we command the PID loop. We do this by
*   first finding the new requested position, then converting this to ticks _based on the startup 0 tick position_. This is similar to
*   absolute positioning vs. relative positioning in Marlin. Every mm->tick calculation needs to be done based on the initial 0 tick position.
*
*/
bool PhotonFeeder::moveInternal(bool forward, uint16_t tenths_mm) {
    signed long goal_mm, timeout, signed_mm, current_tick, output;

    timeout = tenths_mm * TIMEOUT_TIME_PER_TENTH_MM;

    if(!forward){
        signed_mm = tenths_mm * -1;
    }
    else{
        signed_mm = tenths_mm;
    }

    goal_mm = _position + signed_mm;
    // calculating goal_tick based on absolute, not relative position
    float goal_tick_precise = goal_mm * TICKS_PER_TENTH_MM;
    float goal_tick = round(goal_tick_precise);

    unsigned long start_time = millis();

    bool ret = false;

    // float Kp=0.9, Ki=0.4, Kd=20, Hz=120;
    // float Kp=0.9, Ki=0.3, Kd=20, Hz=120;

// works well for 0402 tape at 4mm movements and 2mm movements
// scaling set to 200-255
    // float Kp=0.8, Ki=0.3, Kd=100, Hz=120;

// works 0402 at 2mm movements
    // float Kp=0.9, Ki=0.1, Kd=75, Hz=120;

// works well for 0805 tape at 2mm and 4mm movements
// assumes no film tension with a bit of slack peel release, and static friction comp
    // float Kp=0.9, Ki=0.5, Kd=50, Hz=120;

    //float Kp=0.9, Ki=0.1, Kd=75, Hz=120;


    int output_bits = 8;
    bool output_signed = true;
    FastPID pid(_Kp, _Ki, _Kd, _Hz, output_bits, output_signed);
    pid.setOutputRange(-255, 255);

    int ss_monitor[15] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    int ss_index = 0;
    bool ss;
    bool monatonic = true;


    while(millis() < start_time + timeout){

        current_tick = _encoder->getPosition();

        // check to see if we overshoot outside of the bounds of steady state threshold
        if((current_tick > (goal_tick + SS_THRESHOLD) && forward) || (current_tick < (goal_tick - SS_THRESHOLD) && !forward)){
            ret = false;
            break;
        }

        // updating steady state array with this iteration's error
        signed long error = fabs(goal_tick - current_tick);
        ss_monitor[ss_index] = error;

        // increment steady state array's counter
        if(ss_index >= 14){
            ss_index = 0;
        }
        else{
            ss_index++;
        }

        // setting ss to false if any of the values in ss_monitor are over the threshold
        ss = true;
        for(int i = 0; i<14; i++){
            if(ss_monitor[i] > SS_THRESHOLD){
                ss = false;
                break;
            }
        }

        // if we've hit steady state, return true
        if(ss){
            ret = true;
            //update position to be the new position
            _position = goal_mm;
            break;
        }
        // if we havent, continue to drive the PID
        else{
            // PID implementation
            output = pid.step(goal_tick, current_tick);

            bool forward = true;
            if(output < 0){
                forward = false;
            }

            output = map(abs(output), 0, 255, 70, 255);

            if(output < 71){
                output = 0;
            }

            if(forward){
                analogWrite(_drive1_pin, 0);
                analogWrite(_drive2_pin, output);
            }
            else{
                analogWrite(_drive1_pin, output);
                analogWrite(_drive2_pin, 0);
            }
        }
    }

    // brake to kill any coast
    brakeDrive(100);
    // turn off all motors
    stop();

    // Resetting internal position count so we dont creep up into our 2,147,483,647 limit on the variable
    // We can only do this when the exact tick we move to is a whole number so we don't accrue any drift
    if(floor(goal_tick_precise) == goal_tick_precise){
        setEncoderPosition(0);
        setMmPosition(0);
    }

    return ret;
}

void PhotonFeeder::setEncoderPosition(uint32_t position){
    _encoder->setPosition(position);
}

void PhotonFeeder::setMmPosition(uint16_t position){
    _position = position;
}

bool PhotonFeeder::peel(uint32_t peel_time, bool dir) {
    if(dir){
        //peel film
        digitalWrite(PA8, LOW);
        analogWrite(_peel1_pin, 255);
        analogWrite(_peel2_pin, 0);
        delay(peel_time);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 0);
        digitalWrite(PA8, HIGH);

    }
    else{
        //peel film
        digitalWrite(PA8, LOW);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 255);
        delay(peel_time);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 0);
        digitalWrite(PA8, HIGH);
    }
    return true;
}

void PhotonFeeder::driveTape(bool forward){
    if(forward){
        analogWrite(_drive1_pin, 0);
        analogWrite(_drive2_pin, 255);
    }
    else{
        analogWrite(_drive1_pin, 255);
        analogWrite(_drive2_pin, 0);
    }
}

bool PhotonFeeder::moveForward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    stop();

    // peel film based on how much we're moving
    signed long peel_time = tenths_mm * TENSION_TIME_PER_TENTH_MM;

    peel(peel_time, true);

    // 50ms delay to not get crazy back emf
    delay(50);

    // unpeel just a bit to provide slack, and let it coast for a sec
    peel(100, false);
    delay(50);

    int retry_index = 0;

    if(moveInternal(true, tenths_mm)){ //if moving tape succeeds
        return true;
    }
    else{ // if it fails, try again with a fresh pulse of power after moving the motor back a bit.
        //checkLoaded();
        for(int retry_index = 0; retry_index < _retry_limit; retry_index++){
            driveTape(false);
            delay(50);
            stop();
            if(moveInternal(true, tenths_mm)){
                return true;
            }
        }
    }
    return false;
}

bool PhotonFeeder::moveBackward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    stop();

    // Next, unspool some film to give the tape slack
    signed long peel_time = (tenths_mm * TENSION_TIME_PER_TENTH_MM);
    peel(peel_time, false);

    // move tape backward
    // first we overshoot by the backlash distance, then approach from the forward direction
    moveInternal(false, tenths_mm + BACKLASH_COMP_TENTH_MM);
    moveInternal(true, BACKLASH_COMP_TENTH_MM);

    //peel again to take up any slack
    peel(200, true);

    return true;

}

void PhotonFeeder::stop() {
    // Stop Everything
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 0);
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);
}

void PhotonFeeder::set_rgb(bool red, bool green, bool blue) {
  digitalWrite(LED_R, ! red);
  digitalWrite(LED_G, ! green);
  digitalWrite(LED_B, ! blue);
}

uint8_t PhotonFeeder::read_floor_address() {
  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()) {
    return 0xFF;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // array with the commands to initiate a read, DS28E07 device expect 3 bytes to start a read: command,LSB&MSB adresses
  uint8_t leemem[3] = {
    0xF0,
    0x00,
    0x00
  };

  // sending those three bytes
  _oneWire->write_bytes(leemem, sizeof(leemem), 1);

  uint8_t addr = _oneWire->read();  // Start by reading our address byte

  // Read the next 31 bytes, discarding their value. Each page is 32 bytes so we need 32 read commands
  for (uint8_t i = 0; i < 31; i++) {
    _oneWire->read();
  }

  // return the first byte from returned data
  return addr;
}

bool PhotonFeeder::write_floor_address(uint8_t address) {
/*
    wriıte_floor_address()
      success returns programmed address byte
      failure returns 0xFF

This function takes a byte as in input, and flashes it to address 0x0000 in the eeprom (where the floor ID is stored).
The DS28E07 requires a million and one steps to make this happen. Reference the datasheet for details:
https://datasheets.maximintegrated.com/en/ds/DS28E07.pdf
*/


  byte i;                         // This is for the for loops
  //-----
  // Write To Scratchpad
  //-----

  byte data[8] = {address, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()){
    return 0xFF;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // array with the commands to initiate a write to the scratchpad, DS28E07 device expect 3 bytes to start a read: command,LSB&MSB adresses
  byte leemem[3] = {
    0x0F,
    0x00,
    0x00
  };

  // sending those three bytes
  _oneWire->write_bytes(leemem, sizeof(leemem), 1);

  // Now it's time to actually write the data to the scratchpad
  for ( i = 0; i < 8; i++) {
    _oneWire->write(data[i], 1);
  }

  // read back the CRC
  byte ccrc = _oneWire->read();

  //-----
  // Read Scratchpad
  //-----

  // byte for the ccrc the eeprom will send us
  byte scratchpad_ccrc;

  // reset the 1-wire line, and return failure if no chip detected
  if(!_oneWire->reset()){
    return false;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // send read scratchpad command
  _oneWire->write(0xAA, 1);

  // array for the data we'll read back
  byte read_data[11];

  //read in TA1, TA2, and E/S bytes, then the 8 bytes of data
  for ( i = 0; i < sizeof(read_data); i++) {
    read_data[i] = _oneWire->read();
  }

  //read crc
  scratchpad_ccrc = _oneWire->read();

  byte ccrc_calc = OneWire::crc8(read_data, sizeof(read_data));

  // TODO need to be checking CCRC. never returns true, even when data is identical.
  // if(scratchpad_ccrc != ccrc_calc){
  //   // do nothing
  // }

  //-----
  // Copy Scratchpad to Memory
  //-----

  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()){
    return false;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // copy scratchpad command
  _oneWire->write(0x55, 1);

  // sending auth bytes from scratchpad read, which is the first 3 bytes
  _oneWire->write_bytes(read_data, 3, 1);

  // wait for programming, we'll get alternating 1s and 0s when done
  float timer = millis();
  while(true){
    if(_oneWire->read() == 0xAA){
      break;
    }
    if( (millis() - timer) > 20 ){ // datasheet says it should only ever take 12ms at most to program
      break;
    }
  }

  // send reset
  if(!_oneWire->reset()){
    return false;
  }

  // check the floor address by reading
  byte written_address = this->read_floor_address();

  if(written_address == address) {
    //return new address
    return true;
  }

  return false;
}