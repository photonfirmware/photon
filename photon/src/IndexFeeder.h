#ifndef _INDEX_FEEDER_H
#define _INDEX_FEEDER_H

#include "Feeder.h"
#include <functional>

#ifndef MOTOR_DEPS
    #define MOTOR_DEPS
    #include <RotaryEncoder.h>
    #include <FastPID.h>

#endif 



class IndexFeeder : public Feeder {

    public:
        IndexFeeder(uint8_t drive1_pin, uint8_t drive2_pin, uint8_t peel1_pin, uint8_t peel2_pin, RotaryEncoder* encoder);
        bool init() override;
        Feeder::FeedResult feedDistance(uint16_t tenths_mm, bool forward) override;
        bool peel(uint32_t peel_time, bool dir);
        
    private:
        uint8_t _drive1_pin;
        uint8_t _drive2_pin;

        uint8_t _peel1_pin;
        uint8_t _peel2_pin;  

        signed long _position;

        RotaryEncoder* _encoder;

        double* _Setpoint;
        double* _Input;
        double* _Output;

        bool moveForward(uint16_t tenths_mm);
        bool moveBackward(uint16_t tenths_mm);
        bool moveInternal(bool forward, uint16_t tenths_mm);
        void stop();
        
        bool loopback();

};

#endif //_INDEX_FEEDER_H