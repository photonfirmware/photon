#ifndef _PHOTON_FEEDER_H
#define _PHOTON_FEEDER_H

#include <functional>

#ifndef MOTOR_DEPS
    #define MOTOR_DEPS
    #include <RotaryEncoder.h>
    #include <FastPID.h>
#endif


#define VENDOR_SPECIFIC_OPTIONS_LENGTH 20  // Chosen by fair d20 roll


class PhotonFeeder {
    public:
        enum FeedResult {
            SUCCESS,
            INVALID_LENGTH,
            COULDNT_REACH,
            UNKNOWN_ERROR
        };

        PhotonFeeder(
            uint8_t drive1_pin,
            uint8_t drive2_pin,
            uint8_t peel1_pin,
            uint8_t peel2_pin,
            uint8_t led_red,
            uint8_t led_green,
            uint8_t led_blue,
            RotaryEncoder* encoder
        );

        FeedResult getMoveResult(); 
        
        // Async Functions
        void peel(bool forward);
        void drive(bool forward);
        void brakePeel();
        void brakeDrive();
        void halt();
        void set_rgb(bool red, bool green, bool blue);
        uint16_t calculateExpectedFeedTime(uint8_t distance, bool forward);
        void setMmPosition(uint16_t position); 
        void resetEncoderPosition();

        // Blocking Functions
        void feedDistance(uint16_t tenths_mm, bool forward);
        bool checkLoaded();

        // Vendor Specific Functions
        // Should probably be virtual with hardware overriding it as well as return status
        void vendorSpecific(uint8_t options[VENDOR_SPECIFIC_OPTIONS_LENGTH]) {};
        void identify() {};
        
    private:
        uint8_t _drive1_pin;
        uint8_t _drive2_pin;

        uint8_t _peel1_pin;
        uint8_t _peel2_pin;  

        uint8_t _led_red;
        uint8_t _led_green;
        uint8_t _led_blue;

        uint8_t _retry_limit = 3;

        FeedResult _lastFeedStatus;

        signed long _position;

        RotaryEncoder* _encoder;

        float _Kp=0.5; // higher value, stronger response
        float _Ki=0.01; // higher value, stronger response (divided by Hz)
        float _Kd=2; // higher value, stronger response (multiplied by change in error and Hz)

        int _Hz=100; // higher value, lower I, higher D

        bool moveForward(uint16_t tenths_mm);
        bool moveBackward(uint16_t tenths_mm);
        bool moveInternal(bool forward, uint16_t tenths_mm);
};

#endif //_PHOTON_FEEDER_H