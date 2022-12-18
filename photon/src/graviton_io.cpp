/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "graviton_io.h"
#include <Arduino.h>

/*
    Macros and constants
*/
const static graviton_io_time_t nil_time = 0;

/*
    Forward declarations / inline functions
*/
inline static bool is_nil_time(graviton_io_time_t time) { return time == 0; }
inline static graviton_io_time_t make_timeout_time_ms(uint32_t delay) { return millis() + delay; }
inline static bool time_reached(graviton_io_time_t time) { return time < millis(); }

static int32_t graviton_io_read_impl(struct GravitonIO* io);
static int32_t graviton_io_write_impl(struct GravitonIO* io, uint8_t* data, size_t len);

/*
    Public functions
*/

GravitonIODriver::GravitonIODriver(
    HardwareSerial* serial, uint32_t de, uint32_t re, uint32_t first_byte_timeout_ms, uint32_t total_timeout_ms) {
    this->context.serial = serial;
    this->context.de = de;
    this->context.re = re;
    this->context.first_byte_timeout_ms = first_byte_timeout_ms;
    this->context.total_timeout_ms = total_timeout_ms;
    this->context.first_byte_deadline = nil_time;
    this->context.total_deadline = nil_time;
    this->context.seen_first_byte = false;

    this->io.read = graviton_io_read_impl;
    this->io.write = graviton_io_write_impl;
    this->io.context = &this->context;
}

/*
    Private functions
*/

int32_t graviton_io_read_impl(struct GravitonIO* io) {
    GravitonIODriver::GravitonIODriverContext* ctx = (GravitonIODriver::GravitonIODriverContext*)io->context;

    // The deadline is unset first time this is called for a transaction,
    // this allows setting the deadline at the proper absolute time.
    if (is_nil_time(ctx->first_byte_deadline)) {
        ctx->first_byte_deadline = make_timeout_time_ms(ctx->first_byte_timeout_ms);
    }
    if (is_nil_time(ctx->total_deadline)) {
        ctx->total_deadline = make_timeout_time_ms(ctx->total_timeout_ms);
    }

    if ((!ctx->seen_first_byte && time_reached(ctx->first_byte_deadline)) || time_reached(ctx->total_deadline)) {
        return GRAVITON_IO_ABORT;
    }

    int32_t result = ctx->serial->read();

    ctx->seen_first_byte |= (result == 0x55);

    return result;
}

int32_t graviton_io_write_impl(struct GravitonIO* io, uint8_t* data, size_t len) {
    GravitonIODriver::GravitonIODriverContext* ctx = (GravitonIODriver::GravitonIODriverContext*)io->context;

    digitalWrite(ctx->de, HIGH);
    digitalWrite(ctx->re, HIGH);

    auto bytes_written = ctx->serial->write(data, len);
    ctx->serial->flush();

    digitalWrite(ctx->de, LOW);
    digitalWrite(ctx->re, LOW);
    return bytes_written;
}
