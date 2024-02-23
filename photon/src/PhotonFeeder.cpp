#include <Arduino.h>
#include "PhotonFeeder.h"
#include "define.h"

// Calculating Ticks per Tenth MM
// gearbox has ratio of 1:1030
// encoder has 14 ticks per revolution (7 per channel)
// one full rotation of the output shaft is 14*1030 = 14420 ticks
// divided by 32 teeth is 450.625 ticks per tooth
// divided by 40 tenths of a mm per tooth (4mm) is 11.265625 ticks per tenth mm
// 8.8 microns per tick
#define TICKS_PER_TENTH_MM 11.265625
#define TENTH_MM_PER_PIP 40

// -----------
// Thresholds
// -----------

// number of ticks within requested tick position we should begin halting
#define DRIVE_APPROACH_FINAL_TICKS 400
// encoder ticks before reaching final position to stop peeling film to ensure driving alone sets final position
#define ENSURE_DRIVE_FINAL_TICKS 200
// when moving backwards, how far further backwards past requested position to approach from the back
#define BACKLASH_COMP_TENTH_MM 10
//
#define SHORT_DISTANCE_EASE_IN_TICKS 300

// --------
// Timing
// --------

// how long the peel motor will peel film per tenth mm of tape requested to be driven
#define PEEL_TIME_PER_TENTH_MM 18
// how long the peel motor will peel film per tenth mm of tape requested to be driven, for movements less than 30 tenths
#define SHORT_DISTANCE_PEEL_TIME_PER_TENTH_MM 16
// short amount of time peel motor moves backwards to reduce tension on film after peeling
#define PEEL_BACKOFF_TIME 30
// amount of time we allow for each tenth mm before timeout (in ms)
#define TIMEOUT_TIME_PER_TENTH_MM 15
// after driving backwards, how long do we peel to take up any potential slack in the film
#define BACKWARDS_FEED_FILM_SLACK_REMOVAL_TIME 200

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
            RotaryEncoder* encoder
        ) :
    _drive1_pin(drive1_pin),
    _drive2_pin(drive2_pin),
    _peel1_pin(peel1_pin),
    _peel2_pin(peel2_pin),
    _led_red(led_red),
    _led_green(led_green),
    _led_blue(led_blue),
    _position(0),
    _encoder(encoder) {

    pinMode(_drive1_pin, OUTPUT);
    pinMode(_drive2_pin, OUTPUT);
    pinMode(_peel1_pin, OUTPUT);
    pinMode(_peel2_pin, OUTPUT);

    pinMode(_led_red, OUTPUT);
    pinMode(_led_green, OUTPUT);
    pinMode(_led_blue, OUTPUT);
}

//-----------
//
//    TOOLS
//
//-----------

PhotonFeeder::FeedResult PhotonFeeder::getMoveResult(){
    return _lastFeedStatus;
}

uint16_t PhotonFeeder::calculateExpectedFeedTime(uint8_t distance, bool forward){

    // This command is ONLY for generating a time that we send back to the host
    // calculating timeouts actually determining if we've failed a feed is separate.
    if(forward){
        // we're calculating expected feed time of an _optimal_ forward feed command. this includes:
        // - peel forward time
        // - peel backoff time
        // - expected time to drive forward assuming one attempt
        return (distance * PEEL_TIME_PER_TENTH_MM) + PEEL_BACKOFF_TIME + (distance * TIMEOUT_TIME_PER_TENTH_MM) + 10;
    }
    else {
        // we're calculating expected feed time of an _optimal_ backward feed command. this includes:
        // - unpeeling film time to prep for backwards movement
        // - backwards movement including backlash distance
        // - remaining film slack takeup
        return (distance * PEEL_TIME_PER_TENTH_MM) + ((distance + (BACKLASH_COMP_TENTH_MM*2)) * TIMEOUT_TIME_PER_TENTH_MM) + BACKWARDS_FEED_FILM_SLACK_REMOVAL_TIME + 10;
    }
}

void PhotonFeeder::identify() {
    for(int i = 0;i<3;i++){
        set_rgb(true, true, true);
        delay(300);
        set_rgb(false, false, false);
        delay(300);
    }
    
}

bool PhotonFeeder::checkLoaded() {
/*  checkLoaded()
    The checkLoaded() function checks to see what's loaded in the feeder, and sets PID components appropriately.
    This gets run on first movement command after boot, after quick move, or after the protocol requests it.

*/

    //takes up any backlash slack, ensures any forward movement is tape movement
    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 250);
    delay(2);

    analogWrite(_drive1_pin, 0);
    analogWrite(_drive2_pin, 20);
    delay(100);

    halt();

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

    halt();

    return true;

}

void PhotonFeeder::resetEncoderPosition(uint16_t position){
    _encoder->setPosition(position);
}

void PhotonFeeder::setMmPosition(uint16_t position){
    _position = position;
}

void PhotonFeeder::set_rgb(bool red, bool green, bool blue) {
  digitalWrite(LED_R, ! red);
  digitalWrite(LED_G, ! green);
  digitalWrite(LED_B, ! blue);
}

//-----------
//
//    MANUAL MOTOR CONTROL
//
//-----------

void PhotonFeeder::peel(bool forward) {
    if(forward){
        analogWrite(_peel1_pin, 255);
        analogWrite(_peel2_pin, 0);
    }
    else{
        analogWrite(_peel1_pin, 0);
        analogWrite(_peel2_pin, 255);
    }
}

void PhotonFeeder::drive(bool forward){
    if(forward){
        analogWrite(_drive1_pin, 0);
        analogWrite(_drive2_pin, 255);
    }
    else{
        analogWrite(_drive1_pin, 255);
        analogWrite(_drive2_pin, 0);
    }
}

void PhotonFeeder::driveValue(bool forward, uint8_t value){
    if(forward){
        analogWrite(_drive1_pin, 0);
        analogWrite(_drive2_pin, value);
    }
    else{
        analogWrite(_drive1_pin, value);
        analogWrite(_drive2_pin, 0);
    }
}

void PhotonFeeder::driveBrakeValue(bool forward, uint8_t value){
    if(forward){
        analogWrite(_drive1_pin, 255-value);
        analogWrite(_drive2_pin, 255);
    }
    else{
        analogWrite(_drive1_pin, 255);
        analogWrite(_drive2_pin, 255-value);
    }
}

void PhotonFeeder::brakeDrive(){
    //brings both drive pins high
    analogWrite(_drive1_pin, 255);
    analogWrite(_drive2_pin, 255);
}

void PhotonFeeder::brakePeel(){
    //brings both drive pins high
    analogWrite(_peel1_pin, 255);
    analogWrite(_peel2_pin, 255);
}

void PhotonFeeder::halt(){
    brakeDrive();
    brakePeel();
}

//-----------
//
//    FEEDING
//
//-----------

void PhotonFeeder::feedDistance(uint16_t tenths_mm, bool forward) {


    bool success = (forward) ? moveForward(tenths_mm) : moveBackward(tenths_mm);

}

bool PhotonFeeder::moveForward(uint16_t tenths_mm) {
    int retry_index = 0;
    // First, ensure everything is stopped
    halt();

    // segment our total distance into pip chunks
    div_t result = div(tenths_mm, TENTH_MM_PER_PIP);

    // quot is the number of 1 pip loops we run
    for(int i = 0;i<result.quot;i++){
        // move forward 40 tenths
        if(!moveForwardSequence(40)){ // if it fails, try again with a fresh pulse of power after moving the motor back a bit.
            while(true){
                retry_index++;
                if(retry_index > _retry_limit){
                    _lastFeedStatus = PhotonFeeder::FeedResult::COULDNT_REACH;
                    return false;
                }
                drive(false);
                delay(50);
                halt();
                if(moveForwardSequence(40)){
                    break;
                }
            }
        }
    }

    // rem is the value we drive in a last loop to move any remaining distance less than a pip
    if(result.rem > 0){
        //move forward result.rem
        if(!moveForwardSequence(result.rem)){ // if it fails, try again with a fresh pulse of power after moving the motor back a bit.
            while(true){
                retry_index++;
                if(retry_index > _retry_limit){
                    _lastFeedStatus = PhotonFeeder::FeedResult::COULDNT_REACH;
                    return false;
                }
                drive(false);
                delay(50);
                halt();
                if(moveForwardSequence(result.rem)){
                    break;
                }
            }
        }
    }

    _lastFeedStatus = PhotonFeeder::FeedResult::SUCCESS;
    return true;
    
}

bool PhotonFeeder::moveBackward(uint16_t tenths_mm) {
    // First, ensure everything is stopped
    halt();

    // Next, unspool some film to give the tape slack
    signed long peel_time = (tenths_mm * PEEL_TIME_PER_TENTH_MM);
    peel(false);
    delay(peel_time);
    brakePeel();

    // move tape backward
    // first we overshoot by the backlash distance, then approach from the forward direction
    if (moveBackwardSequence(false, tenths_mm + BACKLASH_COMP_TENTH_MM)){
        if(moveBackwardSequence(true, BACKLASH_COMP_TENTH_MM)){
            //peel again to take up any slack
            peel(true);
            delay(BACKWARDS_FEED_FILM_SLACK_REMOVAL_TIME);
            brakePeel();
            _lastFeedStatus = PhotonFeeder::FeedResult::SUCCESS;
            return true;
        }
        else{
            _lastFeedStatus = PhotonFeeder::FeedResult::COULDNT_REACH;
            return false;
        }        
    }
    else{
        _lastFeedStatus = PhotonFeeder::FeedResult::COULDNT_REACH;
        return false;
    }
}

bool PhotonFeeder::moveForwardSequence(uint16_t tenths_mm) {
/* moveForwardSequence()
*   This function actually handles the translation of the mm movement to driving the motor to the right position based on encoder ticks.

    This function should only be called in increments of tenths_mm. It contains all peeling and driving sequencing needed to get accurate 4 mm movements.
 
*   We can't just calculate the number of ticks we need to move for the given mm movement requested, and increment our tick count by that much.
*   Because the tick count is only ever an approximation of the precise mm position, any rounding done from the mm->tick conversion will
*   result in a significant amount of accrued error.
*
*   Instead, we need to use the _mm_ position as ground truth, and only ever use the ticks as only how we command the PID loop. We do this by
*   first finding the new requested position, then converting this to ticks _based on the startup 0 tick position_. This is similar to
*   absolute positioning vs. relative positioning in Marlin. Every mm->tick calculation needs to be done based on the initial 0 tick position.

    returns true if we reach position within timeout, returns false if we timeout. does NOT set _lastFeedStatus.
*
*/

    signed long goal_mm, timeout, signed_mm, current_tick;

    timeout = tenths_mm * TIMEOUT_TIME_PER_TENTH_MM;
    // doing final position math, and if driving forward, we peel at the same time
    goal_mm = _position + tenths_mm;

    // calculating goal_tick based on absolute, not relative position
    float goal_tick_precise = goal_mm * TICKS_PER_TENTH_MM;
    int goal_tick = round(goal_tick_precise);
    int peel_delay = 0;

    if(tenths_mm < 30){
        peel_delay = SHORT_DISTANCE_PEEL_TIME_PER_TENTH_MM * tenths_mm;
        timeout = timeout + 1000;
    }
    else{
        peel_delay = PEEL_TIME_PER_TENTH_MM * tenths_mm;
    }

    // peel film for calculated time
    peel(true);
    delay(peel_delay);
    peel(false);
    delay(PEEL_BACKOFF_TIME);
    brakePeel();

    // drive forward with ease in
    for(int i=150;i<255;i=i+3){
        driveValue(true, i);
        delay(1);
    }
    

    volatile int stall_detection_ticks[20] = {1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5};
    int stall_detection_ticks_index = 0;
    volatile int delta = 5;

    volatile int currentDriveValue = 1;

    bool stall_detected = false;

    // while we havent exceeded timeout
    unsigned long start_time = millis();
    unsigned long last_stall_position_sample_time = millis();

    unsigned long stallCooldownTime = millis();
    bool stallCooldown;

    driveBrakeValue(true, currentDriveValue);

    while(millis() < start_time + timeout + 20){

    // do any necessary sampling

        //getting encoder position
        current_tick = _encoder->getPosition();
        
        //calculating error
        int error = goal_tick - current_tick;

        //updating stall detection array if it's been a minute
        if(millis() > last_stall_position_sample_time + 1){
            last_stall_position_sample_time = millis();
            stall_detection_ticks[stall_detection_ticks_index] = current_tick;

            if(stall_detection_ticks_index > 18){
                stall_detection_ticks_index = 0;
            }
            else{
                stall_detection_ticks_index++;
            }

            int max = *std::max_element(stall_detection_ticks, stall_detection_ticks + 20);
            int min = *std::min_element(stall_detection_ticks, stall_detection_ticks + 20);
            delta = max-min;

            brakeDrive();

            delta = delta;

        }

        //driving calculated value
        driveBrakeValue(true, currentDriveValue);

        //we've reached the final position!
        if(error < 1){
            
            brakeDrive();
            int brakeTick = _encoder->getPosition();

            while (delta > 0){

                //watching for steady state ticks
                if(millis() > last_stall_position_sample_time + 1){

                    //getting encoder position
                    current_tick = _encoder->getPosition();

                    last_stall_position_sample_time = millis();
                    stall_detection_ticks[stall_detection_ticks_index] = current_tick;

                    if(stall_detection_ticks_index > 18){
                        stall_detection_ticks_index = 0;
                    }
                    else{
                        stall_detection_ticks_index++;
                    }

                    int max = *std::max_element(stall_detection_ticks, stall_detection_ticks + 20);
                    int min = *std::min_element(stall_detection_ticks, stall_detection_ticks + 20);
                    delta = max-min;

                }

            }            

            int ssTick = _encoder->getPosition();

            volatile int drifterror = ssTick - brakeTick;

            _position = goal_mm;

            drifterror = drifterror;

            // Resetting internal position count so we dont creep up into our 2,147,483,647 limit on the variable
            // We can only do this when the exact tick we move to is a whole number so we don't accrue any drift
            if(floor(goal_tick_precise) == goal_tick_precise){
                resetEncoderPosition(_encoder->getPosition() - goal_tick_precise);
                setMmPosition(0);
            }

            volatile int encoderDoubleCheck = _encoder->getPosition();

            
            return true;

        }



        if(stallCooldownTime + 10 < millis()){
            stallCooldown = false;
        }


        //check for stalls
        if(delta <= 5 && stallCooldown == false){ //if stall detected

            currentDriveValue = currentDriveValue + 3;
            if(currentDriveValue > 255){
                currentDriveValue = 255;
            }

            stallCooldown = true;
            stallCooldownTime = millis();

        }
        
    }
    // brake to kill any coast
    halt();

    return false;
}

bool PhotonFeeder::moveBackwardSequence(bool forward, uint16_t tenths_mm) {
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



    signed long goal_mm, timeout, signed_mm, current_tick;

    timeout = tenths_mm * TIMEOUT_TIME_PER_TENTH_MM;

    // doing final position math, and if driving forward, we peel at the same time
    if(forward){
        goal_mm = _position + tenths_mm;
        peel(true);
    }
    else{
        goal_mm = _position - tenths_mm;
    }

    // calculating goal_tick based on absolute, not relative position
    float goal_tick_precise = goal_mm * TICKS_PER_TENTH_MM;
    int goal_tick = round(goal_tick_precise);

    unsigned long start_time = millis();


    // in the direction of the goal with ease in
    for(int i=0;i<255;i=i+5){
        driveValue(forward, i);
        delay(1);
    }
    drive(forward);

    while(millis() < start_time + timeout + 20){

        current_tick = _encoder->getPosition();
        int error;
        
        if(forward){
            error = goal_tick - current_tick;

            // if we're approaching final position, stop peeling to ensure no backlash error
            if(error < ENSURE_DRIVE_FINAL_TICKS){
                brakePeel();
            }

        }
        else{
            error = current_tick - goal_tick;
        }

        if (error < 0){
            brakeDrive();
            _position = goal_mm;

            // Resetting internal position count so we dont creep up into our 2,147,483,647 limit on the variable
            // We can only do this when the exact tick we move to is a whole number so we don't accrue any drift
            if(floor(goal_tick_precise) == goal_tick_precise){
                resetEncoderPosition(current_tick - goal_tick_precise);
                setMmPosition(0);
            }

            return true;
        }

    }
    // brake to kill any coast
    brakeDrive();

    return false;
}
