/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#ifndef _GRAVITON_IO_H
#define _GRAVITON_IO_H

#include "graviton/graviton.h"
#include <HardwareSerial.h>
#include <cstddef>
#include <cstdint>

typedef unsigned long graviton_io_time_t;

class GravitonIODriver {

  public:
    GravitonIODriver(
        HardwareSerial* serial, uint32_t de, uint32_t re, uint32_t first_byte_timeout_ms, uint32_t total_timeout_ms);

    struct GravitonIO io;
    struct GravitonIODriverContext {
        HardwareSerial* serial;
        uint32_t de;
        uint32_t re;
        uint32_t first_byte_timeout_ms;
        uint32_t total_timeout_ms;
        graviton_io_time_t first_byte_deadline;
        graviton_io_time_t total_deadline;
        bool seen_first_byte;
    } context;
};

#endif  //_GRAVITON_IO_H
