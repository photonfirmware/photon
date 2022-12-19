/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

inline static uint8_t graviton_crc8(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t byte_n = 0; byte_n < len; byte_n++) {
        crc ^= (data[byte_n] << 8);
        for (size_t bit_n = 0; bit_n < 8; bit_n++) {
            if (crc & 0x8000)
                crc ^= (0x1070 << 3);
            crc <<= 1;
        }
    }
    return (uint8_t)(crc >> 8);
}

/* Basic 32 byte datagram for sending and receiving messages */
struct GravitonDatagram {
    // Start byte, always 0x55
    uint8_t start;
    // Physical address of the source device
    uint8_t src;
    // Physical address of the destination device
    uint8_t dst;
    // Application-defined protocol ID
    uint8_t protocol;
    // Checksum
    uint8_t crc8;
    // Message data
    uint8_t payload[26];
    // Stop byte, always 0x2A
    uint8_t stop;
} __attribute__((packed));

static_assert(sizeof(struct GravitonDatagram) == 32, "Datagram must be 32 bytes");

#define GRAVITON_DATAGRAM_SIZE (sizeof(struct GravitonDatagram))
#define GRAVITON_PAYLOAD_SIZE                                                                                          \
    (sizeof(((struct GravitonDatagram*)0)->payload) / sizeof(((struct GravitonDatagram*)0)->payload[0]))

/* Initializes a datagram, appropriately setting the start and stop bytes and zeroing the payload. */
inline static void
GravitonDatagram_init(struct GravitonDatagram* datagram, uint8_t src, uint8_t dst, uint8_t protocol) {
    datagram->start = 0x55;
    datagram->src = src;
    datagram->dst = dst;
    datagram->protocol = protocol;
    datagram->crc8 = 0x00;
    memset(datagram->payload, 0x00, GRAVITON_PAYLOAD_SIZE);
    datagram->stop = 0x2A;
}

/* Calculates and stores the datagram's checksum */
inline static void GravitonDatagram_set_crc8(struct GravitonDatagram* datagram) {
    datagram->crc8 = 0x00;
    datagram->crc8 = graviton_crc8((uint8_t*)datagram, sizeof(struct GravitonDatagram));
}

/* Re-calculates and checks the datagram's checksum */
inline static bool GravitonDatagram_check_crc8(struct GravitonDatagram* datagram) {
    uint8_t original_crc8 = datagram->crc8;
    GravitonDatagram_set_crc8(datagram);
    uint8_t calculated_crc8 = datagram->crc8;
    datagram->crc8 = original_crc8;
    return original_crc8 == calculated_crc8;
}

/* Returns a pointer to the datagram's byte representation. */
inline static uint8_t* GravitonDatagram_as_bytes(struct GravitonDatagram* datagram) { return (uint8_t*)(datagram); }

/*  Constructs a datagram from its byte representation.
    bytes* must be at least 32 bytes long, and this function
    creates a copy of them. */
inline static struct GravitonDatagram GravitonDatagram_from_bytes(const uint8_t* bytes) {
    struct GravitonDatagram datagram;
    memcpy(GravitonDatagram_as_bytes(&datagram), bytes, GRAVITON_DATAGRAM_SIZE);
    return datagram;
}

/*  Stream I/O

    These are adapters for using the GravitonDatagram_read_from_stream and
    GravitonDatagram_write_to_stream helper functions. It's not required
    to use these helpers, as you can manually deal with reading and writing
    from whatever I/O using GravitonDatagram_as_bytes() and
    GravitonDatagram_from_bytes().

    This is used by GravitonDatagram_read to read bytes from a serial driver.
    This function should either return a single byte from the serial port,
    GRAVITON_SERIAL_READ_RETRY, or GRAVITON_SERIAL_READ_ABORT. Retry means
    the stream reader will attempt to read from the stream again, abort means
    that the stream reader will abort altogether. Typically retry is used to
    indicate that the serial port doesn't yet have data, whereas abort means
    that some overall timeout was exceeded.

    The void* argument is passed through from GravitonDatagram_read and can be
    used to store any relevant caller-defined state.
*/

enum GravitonIOResult {
    // Indicates that there aren't any bytes ready to be read, so
    // the reader should retry.
    GRAVITON_IO_RETRY = -1,
    // Indicates that either an error or a timeout has occurred, so
    // the reader should give up.
    GRAVITON_IO_ABORT = -2,
};

struct GravitonIO {
    /*  Stream read. Should return a single byte from the stream,
        GRAVITON_IO_RETRY, or GRAVITON_IO_ABORT. */
    int32_t (*read)(struct GravitonIO*);
    /*  Stream write. */
    int32_t (*write)(struct GravitonIO*, uint8_t* data, size_t len);
    /*  User-provided context */
    void* context;
};

inline static int32_t GravitonIO_read_byte(struct GravitonIO* io) {
    while (true) {
        int result = io->read(io);
        if (result == GRAVITON_IO_RETRY) {
            continue;
        } else {
            return result;
        }
    }
}

inline static int32_t GravitonIO_read_until_byte(struct GravitonIO* io, uint8_t marker) {
    while (true) {
        int result = io->read(io);
        if (result == marker) {
            return result;
        } else if (result == GRAVITON_IO_RETRY) {
            continue;
        } else if (result < 0) {
            return result;
        } else {
            continue;
        }
    }
}

enum GravitonDatagramReadResult {
    GRAVITON_READ_OK = 0,
    GRAVITON_READ_NO_START_BYTE = -1,
    GRAVITON_READ_NO_END_BYTE = -2,
    GRAVITON_READ_BAD_CRC8 = -3,
    GRAVITON_READ_ABORTED = -4,
    GRAVITON_READ_UNKNOWN = -5,
    GRAVITON_READ_REQUEST_FAILED = -6,
};

/*  Reads a datagram out of a stream.

    Returns GRAVITON_READ_OK if a valid dataframe was read from the stream.
    Returns GRAVITON_READ_NO_START_BYTE or GRAVITON_READ_NO_END_BYTE if the
    dataframe wasn't detected at all. Returns GRAVITON_READ_BAD_CRC8 if a
    dataframe was found but had an invalid checksum. Returns
    GRAVITON_READ_ABORTED if io->read() returned GRAVITON_IO_ABORT while
    reading bytes from the stream. Returns GRAVITON_READ_UNKNOWN under all
    other circumstances.
*/
inline static enum GravitonDatagramReadResult
GravitonDatagram_read_from_stream(struct GravitonDatagram* datagram, struct GravitonIO* io) {
    if (GravitonIO_read_until_byte(io, 0x55) < 0) {
        return GRAVITON_READ_NO_START_BYTE;
    }

    datagram->start = 0x55;

    // Read remaining 31 datagram bytes.
    uint8_t* datagram_bytes = (uint8_t*)(datagram);
    for (size_t i = 1; i < GRAVITON_DATAGRAM_SIZE; i++) {
        int32_t byte = GravitonIO_read_byte(io);
        if (byte < 0) {
            return byte == GRAVITON_IO_ABORT ? GRAVITON_READ_ABORTED : GRAVITON_READ_UNKNOWN;
        }
        datagram_bytes[i] = (uint8_t)(byte);
    }

    // Check for end byte
    if (datagram->stop != 0x2A) {
        return GRAVITON_READ_NO_END_BYTE;
    }

    // Check CRC8
    if (!GravitonDatagram_check_crc8(datagram)) {
        return GRAVITON_READ_BAD_CRC8;
    };

    return GRAVITON_READ_OK;
}

inline static int32_t GravitonDatagram_write_to_stream(struct GravitonDatagram* datagram, struct GravitonIO* io) {
    return io->write(io, GravitonDatagram_as_bytes(datagram), 32);
}

#undef __GRAVITON_READ_STREAM

#ifdef __cplusplus
}
#endif
