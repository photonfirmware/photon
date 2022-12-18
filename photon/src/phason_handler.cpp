/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "phason_handler.h"
#include "define.h"
#include "graviton_io.h"

void PhasonHandler::dispatch() {
    if (!_serial->available()) {
        return;
    }

    GravitonIODriver io_driver = GravitonIODriver(_serial, DE, _RE, 2, 50);
    struct GravitonDatagram req_datagram;

    if (GravitonDatagram_read_from_stream(&req_datagram, &io_driver.io) != GRAVITON_READ_OK) {
        return;
    }

    if (req_datagram.dst != _address) {
        return;
    }

    struct PhasonRequest req = PhasonRequest_from_datagram(&req_datagram);

    switch (req.command) {
        case PHASON_FEEDER_INFO_REQ: {
            struct PhasonFeederInfoResponse resp = {
                .command = PHASON_FEEDER_INFO_RESP,
                .status = PHASON_OK,
                .protocol_version = 1,
                .firmware_year = 22,
                .firmware_month = 12,
                .firmware_day = 17,
            };
            memcpy(resp.uid, _uid, PHASON_UID_SIZE);
            phason_send_response(&io_driver.io, _address, (const PhasonResponse*)&resp);
        } break;

        case PHASON_RESET_FEEDER_REQ: {
            bool succeeded = _feeder->init();
            _sequence = 0;

            struct PhasonResponse resp = {
                .command = PHASON_RESET_FEEDER_RESP,
                .status = succeeded ? PHASON_OK : PHASON_ERROR,
            };
            phason_send_response(&io_driver.io, _address, &resp);
        } break;

        case PHASON_START_FEED_REQ: {
            struct PhasonStartFeedRequest start_feed_req = PhasonStartFeedRequest_from_datagram(&req_datagram);

            // Make sure the sequence isn't the same as the last feed.
            if (start_feed_req.sequence == _sequence) {
                struct PhasonStartFeedResponse resp = {
                    .command = PHASON_START_FEED_RESP,
                    .status = PHASON_INVALID_REQUEST,
                    .sequence = _sequence,
                };
                phason_send_response(&io_driver.io, _address, (struct PhasonResponse*)&resp);
            } else {
                _sequence = start_feed_req.sequence;

                auto direction = start_feed_req.micrometers > 0;
                auto abs_tenths_mm = abs(start_feed_req.micrometers) / 100;

                // Note: feedDistance is currently blocking, so this always
                // accepts the move as long as the sequence numbers are okay.
                struct PhasonStartFeedResponse resp = {
                    .command = PHASON_START_FEED_RESP,
                    .status = PHASON_OK,
                    .sequence = _sequence,
                };
                phason_send_response(&io_driver.io, _address, (struct PhasonResponse*)&resp);

                _last_feed_status = _feeder->feedDistance(abs_tenths_mm, direction) == Feeder::FeedResult::SUCCESS;
                _last_feed_micrometers = abs_tenths_mm * 100;
            }
        } break;

        case PHASON_FEED_STATUS_REQ: {
            struct PhasonFeedStatusRequest feed_status_req = PhasonFeedStatusRequest_from_datagram(&req_datagram);

            // Make sure the sequence number matches
            if (feed_status_req.sequence != _sequence) {
                struct PhasonFeedStatusResponse resp = {
                    .command = PHASON_FEED_STATUS_RESP,
                    .status = PHASON_INVALID_REQUEST,
                    .sequence = _sequence,
                };
                phason_send_response(&io_driver.io, _address, (struct PhasonResponse*)&resp);
            } else {
                struct PhasonFeedStatusResponse resp = {
                    .command = PHASON_FEED_STATUS_RESP,
                    .status = _last_feed_status ? PHASON_OK : PHASON_MOTOR_ERROR,
                    .sequence = _sequence,
                    .actual_micrometers = _last_feed_micrometers,
                };
                phason_send_response(&io_driver.io, _address, (struct PhasonResponse*)&resp);
            }
        } break;

        default: {
            struct PhasonResponse resp = {
                .command = PHASON_RESET_FEEDER_RESP,
                .status = PHASON_INVALID_REQUEST,
            };
            phason_send_response(&io_driver.io, _address, &resp);
        } break;
    }
}
