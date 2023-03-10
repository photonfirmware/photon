#include "PhotonFeederProtocol.h"
#include "PhotonNetworkLayer.h"

#include "rs485/protocols/checksums/crc8_107.h"

#define MAX_PROTOCOL_VERSION 1

typedef enum {
    STATUS_OK = 0x00,
    STATUS_WRONG_FEEDER_ID = 0x01,
    STATUS_COULDNT_REACH = 0x02,
    STATUS_UNINITIALIZED_FEEDER = 0x03,
    STATUS_FEEDING_IN_PROGRESS = 0x04,
    STATUS_FAIL = 0x05,
    
    STATUS_TIMEOUT = 0xFE,
    STATUS_UNKNOWN_ERROR = 0xFF
} FeederStatus;

typedef enum {
    // Unicast Commands
    GET_FEEDER_ID = 0x01,
    INITIALIZE_FEEDER = 0x02, 
    GET_VERSION = 0x03,
    MOVE_FEED_FORWARD = 0x04,
    MOVE_FEED_BACKWARD = 0x05,
    MOVE_FEED_STATUS = 0x06,

    VENDOR_OPTIONS = 0xbf,

    // Broadcast Commands
    GET_FEEDER_ADDRESS = 0xc0,
    IDENTIFY_FEEDER = 0xc1,
    PROGRAM_FEEDER_FLOOR = 0xc2,
    // EXTENDED_COMMAND = 0xff, Unused, reserved for future use
} FeederCommand;

PhotonFeederProtocol::PhotonFeederProtocol(
    PhotonFeeder *feeder,
    FeederFloor *feederFloor,
    PhotonNetworkLayer* network,
    const uint8_t *uuid, size_t uuid_length) :
    _feeder(feeder),
    _feederFloor(feederFloor),
    _network(network),
    _initialized(false) {
    memset(_uuid, 0, UUID_LENGTH);
    memcpy(_uuid, uuid, (uuid_length < UUID_LENGTH) ? uuid_length : UUID_LENGTH);
}

void PhotonFeederProtocol::tick() {
    bool hasPacket = _network->getPacket(commandBuffer, sizeof(commandBuffer));

    if(! hasPacket) {
        return;
    }

    switch(command.commandId) {
    //switch(packetBuffer.packet.payload.command.commandId) {
    case GET_FEEDER_ID:
        handleGetFeederId();
        break;
    case INITIALIZE_FEEDER:
        handleInitializeFeeder();
        break;
    case GET_VERSION:
        handleGetVersion();
        break;
    case MOVE_FEED_FORWARD:
        handleMoveFeedForward();
        break;
    case MOVE_FEED_BACKWARD:
        handleMoveFeedBackward();
        break;
    case MOVE_FEED_STATUS:
        handleMoveFeedStatus();
        break;
    case VENDOR_OPTIONS:
        handleVendorOptions();
        break;
    case GET_FEEDER_ADDRESS:
        handleGetFeederAddress();
        break;
    case IDENTIFY_FEEDER:
        handleIdentifyFeeder();
        break;
    case PROGRAM_FEEDER_FLOOR:
        handleProgramFeederFloor();
        break;
    default:
        // Something has gone wrong if execution ever gets here.
        break;
    }
}

bool PhotonFeederProtocol::isInitialized() {
    return _initialized;
}

bool PhotonFeederProtocol::guardInitialized() {
    // Checks if the feeder is initialized and returns an error if it hasn't been
    if (_initialized) {
        return true;
    }

    response = {
        .status = STATUS_UNINITIALIZED_FEEDER,
    };
    memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(InitializeFeederResponse));

    return false;
}


void PhotonFeederProtocol::handleGetFeederId() {
    response = {
        .status = STATUS_OK
    };
    memcpy(response.getFeederId.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(GetFeederIdResponse));
}

void PhotonFeederProtocol::handleInitializeFeeder() {
    // Check uuid is correct, if not return a Wrong Feeder UUID error
    bool requestedUUIDMatchesMine = memcmp(command.initializeFeeder.uuid, _uuid, UUID_LENGTH) == 0;
    if (! requestedUUIDMatchesMine) {
        response = {
            .status = STATUS_WRONG_FEEDER_ID,
        };
        memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

        transmitResponse(sizeof(InitializeFeederResponse));
        return;
    }
    
    // Mark this feeder as initialized
    _initialized = true;

    response = {
        .status = STATUS_OK,
    };
    memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(InitializeFeederResponse));
}

void PhotonFeederProtocol::handleGetVersion() {
    response = {
        .status = STATUS_OK,
        .protocolVersion = {
            .version = MAX_PROTOCOL_VERSION,
        },
    };

    transmitResponse(sizeof(GetProtocolVersionResponse));
}

void PhotonFeederProtocol::move(uint8_t distance, bool forward) {
    if (! guardInitialized()) {
        return;
    }

    uint16_t time = _feeder->calculateExpectedFeedTime(distance, forward);

    response = {
        .status = STATUS_OK,
        .expectedTimeToFeed = {
            .expectedFeedTime = time,
        },
    };

    transmitResponse();

    _feeder->feedDistance(distance, forward);

    _network->clearPackets();

}

void PhotonFeederProtocol::handleMoveFeedForward() {
    move(command.move.distance, true);

}

void PhotonFeederProtocol::handleMoveFeedBackward() {
    move(command.move.distance, false);

}

void PhotonFeederProtocol::handleMoveFeedStatus() {
    PhotonFeeder::FeedResult result = _feeder->getMoveResult();
    uint8_t moveResponseStatus;

    switch (result)
    {
    case PhotonFeeder::FeedResult::SUCCESS:
        moveResponseStatus = STATUS_OK;
        break;
    
    case PhotonFeeder::FeedResult::INVALID_LENGTH:    // For Now Handle Invalid Length & Motor Fault The Same
    case PhotonFeeder::FeedResult::COULDNT_REACH:
        moveResponseStatus = STATUS_COULDNT_REACH;
        break;

    case PhotonFeeder::FeedResult::UNKNOWN_ERROR:
    default:
        moveResponseStatus = STATUS_UNKNOWN_ERROR;
        break;
    }

    response = {
        .status = moveResponseStatus,
    };

    transmitResponse();
}

void PhotonFeederProtocol::handleGetFeederAddress() {
    // Check For Feeder Match
    bool requestedUUIDMatchesMine = memcmp(command.initializeFeeder.uuid, _uuid, UUID_LENGTH) == 0;
    if (! requestedUUIDMatchesMine) {
        return;
    }

    response = {
        .status = STATUS_OK,
    };

    transmitResponse(sizeof(GetFeederAddressResponse));
}

void PhotonFeederProtocol::handleVendorOptions() {
    if (! guardInitialized()) {
        return;
    }

    _feeder->vendorSpecific(command.vendorOptions.options);

    response = {
        .status = STATUS_OK,
    };

    transmitResponse();
}

void PhotonFeederProtocol::handleIdentifyFeeder() {
    _feeder->identify();

    response = {
        .status = STATUS_OK,
    };

    transmitResponse();
}

void PhotonFeederProtocol::handleProgramFeederFloor() {
    bool addressWritten = _feederFloor->write_floor_address(command.programFeederFloorAddress.address);

    uint8_t feederStatus = addressWritten ? STATUS_OK : STATUS_FAIL;
    response = {
        .status = feederStatus,
    };

    transmitResponse();
}

void PhotonFeederProtocol::transmitResponse(uint8_t responseSize) {
    // responseSize is only the anonymous uninion in the response, not any variables common to all responses
    // We handle common values here.
    responseSize++; // uint8_t status

    response.header = {
            .toAddress = PHOTON_NETWORK_CONTROLLER_ADDRESS,
            .fromAddress = _network->getLocalAddress(),
            .packetId = command.header.packetId,
            .payloadLength = responseSize,
    };

    CRC8_107 crc;

    crc.add(response.header.toAddress);
    crc.add(response.header.fromAddress);
    crc.add(response.header.packetId);
    crc.add(response.header.payloadLength);

    for(uint8_t i = 0; i<response.header.payloadLength; i++){
        crc.add(responseBuffer[sizeof(PhotonPacketHeader) + i]);
    }

    response.header.crc = crc;

    size_t totalPacketLength = sizeof(PhotonPacketHeader) + response.header.payloadLength;
    _network->transmitPacket(responseBuffer, totalPacketLength);
}