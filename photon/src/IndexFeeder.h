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
        void driveTape(bool forward);
        void brakeDrive(uint16_t brake_time);
        bool checkLoaded();
        void stop();
        void setMmPosition(uint16_t position);
        void setEncoderPosition(uint32_t position);
        
    private:
        uint8_t _drive1_pin;
        uint8_t _drive2_pin;

        uint8_t _peel1_pin;
        uint8_t _peel2_pin;  

        uint8_t _retry_limit = 3;

        signed long _position;

        RotaryEncoder* _encoder;

        double* _Setpoint;
        double* _Input;
        double* _Output;

        float _Kp=0.5; // higher value, stronger response
        float _Ki=0.01; // higher value, stronger response (divided by Hz)
        float _Kd=2; // higher value, stronger response (multiplied by change in error and Hz)

        int _Hz=100; // higher value, lower I, higher D

        bool moveForward(uint16_t tenths_mm);
        bool moveBackward(uint16_t tenths_mm);
        bool moveInternal(bool forward, uint16_t tenths_mem);
        
        
        bool loopback();

};

#endif //_INDEX_FEEDER_H