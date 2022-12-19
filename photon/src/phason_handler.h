/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#ifndef _PHASON_HANDLER_H

#include "Feeder.h"
#include "graviton/phason.h"
#include <HardwareSerial.h>
#include <cstddef>
#include <cstdint>

class PhasonHandler {
  public:
    PhasonHandler(HardwareSerial* serial, Feeder* feeder, uint8_t address, const uint8_t* uid)
        : _serial(serial), _feeder(feeder), _address(address) {
        memcpy(_uid, uid, PHASON_UID_SIZE);
    };

    void setAddress(uint8_t address) { _address = address; }

    void dispatch();

  protected:
    HardwareSerial* _serial;
    Feeder* _feeder;
    uint8_t _address;
    uint8_t _uid[PHASON_UID_SIZE];
    uint8_t _sequence = 0;
    int32_t _last_feed_micrometers = 0;
    bool _last_feed_status = false;
};

#endif  //_PHASON_HANDLER_H
