#ifndef _FEEDER_H
#define _FEEDER_H

#include <cstddef>
#include <cstdint>

class Feeder {

    public:
        enum FeedResult
        {
            SUCCESS,
            INVALID_LENGTH,
            MOTOR_FAULT,
            UNKNOWN_ERROR
        };

        virtual FeedResult feedDistance(uint16_t tenths_mm, bool forward) = 0;

};

#endif //_FEEDER_H