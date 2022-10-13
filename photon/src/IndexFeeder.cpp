#include <Arduino.h>
#include "IndexFeeder.h"

#define TENTH_MM_PER_PIP 40
#define TIMEOUT_PER_PIP

// drive motor data
// gearbox has ratio of 1:1030
// encoder has 14 ticks per revolution
// one full rotation of the output shaft is 14*1030 = 14420 ticks
// divided by 32 teeth is 450.625 ticks per tooth
// divided by 40 tenths of a mm per tooth (4mm) is 11.265625 ticks per tenth mm
#define TICKS_PER_TENTH_MM 11.265625

#define TENSION_TIMEOUT 400
#define TENSION_DELAY 800
#define BACKWARD_FILM_SLACK_TIMEOUT 200
#define SS_THRESHOLD 3

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255

// Unit Tests Fail Because This Isn't Defined In ArduinoFake for some reason
#ifndef INPUT_ANALOG
#define INPUT_ANALOG 0x04
#endif

IndexFeeder::IndexFeeder(uint8_t drive1_pin, uint8_t drive2_pin, uint8_t peel1_pin, uint8_t peel2_pin, RotaryEncoder* encoder) :
    _drive1_pin(drive1_pin),
    _drive2_pin(drive2_pin),
    _peel1_pin(peel1_pin),
    _peel2_pin(peel2_pin),
    _encoder(encoder),
    _position(0) {
    init();
}

bool IndexFeeder::init() {

    pinMode(_drive1_pin, OUTPUT);
    pinMode(_drive2_pin, OUTPUT);
    pinMode(_peel1_pin, OUTPUT);
    pinMode(_peel2_pin, OUTPUT);

    return true;
}

Feeder::FeedResult IndexFeeder::feedDistance(uint16_t tenths_mm, bool forward) {

    // if (abs(tenths_mm) % TENTH_MM_PER_PIP != 0) {
    //     // The Index Feeder has only been tested and calibrated for moves of 4mm (One Pip) so far.
    //     // If any other value is supplied, indicate it is invalid.
    //     return Feeder::FeedResult::INVALID_LENGTH;
    // }

    bool success = (forward) ? moveForward(tenths_mm) : moveBackward(tenths_mm);
    if (!success) {
        return FeedResult::MOTOR_FAULT;
    }
    

    return Feeder::FeedResult::SUCCESS;
}

//returns true if we reached position within timeout, false if we didn't
bool IndexFeeder::moveInternal(uint32_t timeout, bool forward, uint16_t tenths_mm) {
    signed long goal_mm;

    if(!forward){
        tenths_mm = tenths_mm * -1;
    }

    // updating position
    goal_mm = _position + tenths_mm;
    float goal_ticks = goal_mm * TICKS_PER_TENTH_MM;

    unsigned long start_time = millis();

    bool ret = false;

    // float Kp=200, Ki=5, Kd=0, Hz=60;
    float Kp=2, Ki=5, Kd=0.2, Hz=60;
    int output_bits = 8;
    bool output_signed = true;
    FastPID pid(Kp, Ki, Kd, Hz, output_bits, output_signed);
    pid.setOutputRange(-255, 255);

    int ss_monitor[10] = {100, 100, 100, 100, 100};
    int ss_index = 0;

    while(millis() < start_time + timeout){

        signed long current_pos = _encoder->getPosition();        

        // updating steady state array with this iteration's error
        signed long error = fabs(goal_pos - current_pos);
        ss_monitor[ss_index] = error;

        // increment steady state array's counter
        if(ss_index >= 4){
            ss_index = 0;
        }
        else{
            ss_index++;
        }

        // setting ss to false if any of the values in ss_monitor are over the threshold
        bool ss = true;
        for(int i = 0; i<5; i++){
            if(ss_monitor[i] > 2){
                ss = false;
                break;
            }
        }

        if(ss){
            ret = true;
            break;
        }
        else{
            // PID implementation
            signed long output = pid.step(goal_pos, current_pos);

            if(output > 0){
                analogWrite(_drive1_pin, 0);
                analogWrite(_drive2_pin, output);
            }
            else {
                output = abs(output);
                analogWrite(_drive1_pin, output);
                analogWrite(_drive2_pin, 0);
            }
        }
    }

    // be sure to turn off motors
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 0);

    // TODO: find some way of resetting encoder position so we dont creep up into our 2,147,483,647 limit on the variable
    // running _encoder->setPosition(0) after every movement would accumulate too much error

    return ret;
}

bool IndexFeeder::tension(uint32_t timeout) {
    unsigned long start_millis, current_millis;

    //tension film
    digitalWrite(PA8, LOW);
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 200);
    delay(TENSION_DELAY);
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);
    digitalWrite(PA8, HIGH);


    return true;
}

bool IndexFeeder::moveForward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    stop();
    
    //turn on peel motor
    analogWrite(_peel1_pin, 255);
    analogWrite(_peel2_pin, 0);
    
    //move tape
    moveInternal(10000, true, tenths_mm);

    //turn off peel motor
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);

    return true;
}

bool IndexFeeder::moveBackward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    stop();

    // Next, unspool some film to give the tape slack. imprecise amount because we retention later
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 255);

    //allow reverse peel to happen
    delay(BACKWARD_FILM_SLACK_TIMEOUT);

    //stop reverse peeling
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);

    // move tape backward
    // TODO backlash comp needs to be added here
    moveInternal(5000, false, tenths_mm);

    //peel again
    analogWrite(_peel1_pin, 255);
    analogWrite(_peel2_pin, 0);

    delay(200);

    stop();

    return true;

}

void IndexFeeder::stop() {
    // Stop Everything
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 0);
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);
}