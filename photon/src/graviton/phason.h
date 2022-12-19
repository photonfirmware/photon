/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#include "graviton.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
    Macros and constants
*/

#define PHASON_PROTOCOL_ID 0xA1
#define PHASON_CONTROLLER_ADDRESS 0x00
#define PHASON_MAX_FEEDER_ADDRESS 0x80
#define PHASON_LOGICAL_ADDRESS 0xFF
#define PHASON_UID_SIZE 12

#define __PHASON_ASSERT_PAYLOAD_SIZE(typename)                                                                         \
    static_assert(sizeof(typename) == GRAVITON_PAYLOAD_SIZE, #typename "must be fit in Graviton datagram payload");

/*
    Base Request and Response definitions
*/

enum PhasonStatusCode {
    // All is good. :)
    PHASON_OK = 0,
    // General error code, use only when other codes don't apply.
    PHASON_ERROR = 1,
    // Unknown or malformed request.
    PHASON_INVALID_REQUEST = 2,
    // An issue occurred with the motor.
    PHASON_MOTOR_ERROR = 3,
    // Not ready to response. Retry the request after a delay.
    PHASON_NOT_READY = 4,
};

struct PhasonRequest {
    uint8_t command;  // MSB always clear.
    uint8_t data[25];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonRequest);

struct PhasonResponse {
    uint8_t command;  // MSB always set.
    uint8_t status;
    uint8_t data[24];
} __attribute__((packed));

/*
    Helpers for converting requests & responses to and from datagrams.
*/

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonResponse);

#define __PHASON_FROM_DATAGRAM(typename)                                                                               \
    inline static struct typename typename##_from_datagram(const struct GravitonDatagram* datagram) {                  \
        return *((const struct typename*)(datagram->payload));                                                         \
    }

__PHASON_FROM_DATAGRAM(PhasonRequest);

inline static struct GravitonDatagram PhasonRequest_to_datagram(uint8_t feeder_addr, const void* req) {
    struct GravitonDatagram datagram;
    GravitonDatagram_init(&datagram, PHASON_CONTROLLER_ADDRESS, feeder_addr, PHASON_PROTOCOL_ID);
    memcpy(datagram.payload, (const uint8_t*)(req), GRAVITON_PAYLOAD_SIZE);
    GravitonDatagram_set_crc8(&datagram);
    return datagram;
}

__PHASON_FROM_DATAGRAM(PhasonResponse);

inline static struct GravitonDatagram PhasonResponse_to_datagram(uint8_t feeder_addr, const void* resp) {
    struct GravitonDatagram datagram;
    GravitonDatagram_init(&datagram, feeder_addr, PHASON_CONTROLLER_ADDRESS, PHASON_PROTOCOL_ID);
    memcpy(datagram.payload, (const uint8_t*)(resp), GRAVITON_PAYLOAD_SIZE);
    GravitonDatagram_set_crc8(&datagram);
    return datagram;
}

/*
    Command specific request and response definitions
*/

enum PhasonCommands {
    PHASON_FEEDER_INFO_REQ = 0x01,
    PHASON_FEEDER_INFO_RESP = 0x81,
    PHASON_RESET_FEEDER_REQ = 0x02,
    PHASON_RESET_FEEDER_RESP = 0x82,
    PHASON_START_FEED_REQ = 0x03,
    PHASON_START_FEED_RESP = 0x83,
    PHASON_FEED_STATUS_REQ = 0x04,
    PHASON_FEED_STATUS_RESP = 0x84,
    PHASON_QUERY_BY_UID_REQ = 0x04,
    PHASON_QUERY_BY_UID_RESP = 0x84,
};

struct PhasonFeederInfoResponse {
    uint8_t command;  // Always 0x81
    uint8_t status;
    uint8_t protocol_version;
    uint8_t firmware_year;
    uint8_t firmware_month;
    uint8_t firmware_day;
    uint8_t uid[12];
    uint8_t padding[8];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonFeederInfoResponse);
__PHASON_FROM_DATAGRAM(PhasonFeederInfoResponse);

struct PhasonStartFeedRequest {
    uint8_t command;  // Always 0x03
    uint8_t sequence;
    int32_t micrometers;
    uint8_t padding[20];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonStartFeedRequest);
__PHASON_FROM_DATAGRAM(PhasonStartFeedRequest);

struct PhasonStartFeedResponse {
    uint8_t command;  // Always 0x83
    uint8_t status;
    uint8_t sequence;
    uint8_t padding[23];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonStartFeedResponse);
__PHASON_FROM_DATAGRAM(PhasonStartFeedResponse);

struct PhasonFeedStatusRequest {
    uint8_t command;  // Always 0x04
    uint8_t sequence;
    uint8_t padding[24];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonFeedStatusRequest);
__PHASON_FROM_DATAGRAM(PhasonFeedStatusRequest);

struct PhasonFeedStatusResponse {
    uint8_t command;  // Always 0x84
    uint8_t status;
    uint8_t sequence;
    int32_t actual_micrometers;
    uint8_t padding[19];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonFeedStatusResponse);
__PHASON_FROM_DATAGRAM(PhasonFeedStatusResponse);

struct PhasonQueryByUIDRequest {
    uint8_t command;  // Always 0x1F
    uint8_t uid[12];
    uint8_t padding[13];
} __attribute__((packed));

__PHASON_ASSERT_PAYLOAD_SIZE(struct PhasonQueryByUIDRequest);

/*
    Helpers for communications
*/

/* Send a request and read a reply */
inline static enum GravitonDatagramReadResult phason_send_request(
    struct GravitonIO* io, uint8_t feeder_addr, const struct PhasonRequest* req, struct GravitonDatagram* resp) {
    struct GravitonDatagram request_datagram = PhasonRequest_to_datagram(feeder_addr, req);
    if (GravitonDatagram_write_to_stream(&request_datagram, io) < 0) {
        return GRAVITON_READ_REQUEST_FAILED;
    };
    return GravitonDatagram_read_from_stream(resp, io);
}

#define __PHASON_SEND_REQUEST(req_name, resp_typename)                                                                 \
    inline static enum GravitonDatagramReadResult phason_send_##req_name##_request(                                    \
        struct GravitonIO* io, uint8_t feeder_addr, const struct PhasonRequest* req, struct resp_typename* resp) {     \
        struct GravitonDatagram resp_dg;                                                                               \
        int32_t result = phason_send_request(io, feeder_addr, req, &resp_dg);                                          \
        if (result != GRAVITON_READ_OK) {                                                                              \
            return (enum GravitonDatagramReadResult)result;                                                                                             \
        }                                                                                                              \
        (*resp) = resp_typename##_from_datagram(&resp_dg);                                                             \
        return (enum GravitonDatagramReadResult)result;                                                                                                 \
    }

__PHASON_SEND_REQUEST(feeder_info, PhasonFeederInfoResponse);
__PHASON_SEND_REQUEST(reset_feeder, PhasonResponse);
__PHASON_SEND_REQUEST(start_feed, PhasonStartFeedResponse);
__PHASON_SEND_REQUEST(feed_status, PhasonFeedStatusResponse);

/* Send a response */
inline static int32_t
phason_send_response(struct GravitonIO* io, uint8_t feeder_addr, const struct PhasonResponse* resp) {
    struct GravitonDatagram response_datagram = PhasonResponse_to_datagram(feeder_addr, resp);
    return GravitonDatagram_write_to_stream(&response_datagram, io);
}

#ifdef __cplusplus
}
#endif
