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
#define TENSION_TIME_PER_TENTH_MM 17
#define TIMEOUT_TIME_PER_TENTH_MM 35

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
bool IndexFeeder::moveInternal(bool forward, uint16_t tenths_mm) {
    signed long goal_mm, timeout, signed_mm;

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

    // float Kp=200, Ki=5, Kd=0, Hz=60;
    float Kp=2, Ki=5, Kd=0.2, Hz=60;
    int output_bits = 8;
    bool output_signed = true;
    FastPID pid(Kp, Ki, Kd, Hz, output_bits, output_signed);
    pid.setOutputRange(-255, 255);

    int ss_monitor[5] = {100, 100, 100, 100, 100};
    int ss_index = 0;

    while(millis() < start_time + timeout){

        signed long current_tick = _encoder->getPosition();        

        // updating steady state array with this iteration's error
        signed long error = fabs(goal_tick - current_tick);
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
            signed long output = pid.step(goal_tick, current_tick);

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
    stop();

    // Resetting internal position count so we dont creep up into our 2,147,483,647 limit on the variable
    // We can only do this when the exact tick we move to is a whole number so we don't accrue any drift
    if(floor(goal_tick_precise) == goal_tick_precise){
        _position = 0;
        _encoder->setPosition(0);
    }

    return ret;
}

bool IndexFeeder::peel(uint32_t peel_time, bool dir) {
    if(dir){
        //peel film
        digitalWrite(PA8, LOW);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 255);
        delay(peel_time);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 0);
        digitalWrite(PA8, HIGH);

    }
    else{
        //peel film
        digitalWrite(PA8, LOW);
        analogWrite(_peel1_pin, 255);
        analogWrite(_peel2_pin, 0);
        delay(peel_time);
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 0);
        digitalWrite(PA8, HIGH);
    }
    return true;
}

bool IndexFeeder::moveForward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    stop();
    
    //turn on peel motor
    signed long peel_time = tenths_mm * TENSION_TIME_PER_TENTH_MM;
    peel(peel_time, true);

    //start timer before moving tape
    unsigned long start = millis();
    
    //move tape
    moveInternal(true, tenths_mm);

    //calculate how long we drove tape for
    unsigned long drive_time = millis() - start;

    // //peel for any remaining time needed, if any
    // signed long peel_time = (tenths_mm * TENSION_TIME_PER_TENTH_MM) - drive_time;
    // if(peel_time < 0){
    //     peel_time = 0;
    // }
    // peel(peel_time, true);

    return true;
}

bool IndexFeeder::moveBackward(uint16_t tenths_mm) {
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

void IndexFeeder::stop() {
    // Stop Everything
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 0);
    analogWrite(_peel1_pin, 0);
    analogWrite(_peel2_pin, 0);
}