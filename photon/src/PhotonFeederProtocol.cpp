#include "PhotonFeederProtocol.h"
#include "PhotonNetworkLayer.h"

#define MAX_PROTOCOL_VERSION 1

typedef enum {
    STATUS_OK = 0x00,
    STATUS_WRONG_FEEDER_ID = 0x01,
    STATUS_MOTOR_FAULT = 0x02,
    STATUS_UNINITIALIZED_FEEDER = 0x03,
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

    // Broadcast Commands
    GET_FEEDER_ADDRESS = 0xc0,
} FeederCommand;

PhotonFeederProtocol::PhotonFeederProtocol(
    PhotonFeeder *feeder,
    PhotonNetworkLayer* network,
    const uint8_t *uuid, size_t uuid_length) : _feeder(feeder), _network(network), _initialized(false) {
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
    case GET_FEEDER_ADDRESS:
        handleGetFeederAddress();
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
    // Response: <feeder address> 0x03 <uuid:12>

    if (_initialized) {
        return true;
    }

    response = {
        .initializeFeeder = {
            .status = STATUS_UNINITIALIZED_FEEDER,
        },
    };
    memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(InitializeFeederResponse));

    return false;
}


void PhotonFeederProtocol::handleGetFeederId() {
    // Payload: <command id>
    // Response: <feeder address> <ok> <uuid:12>

    response = {
        .getFeederId = {
            .status = STATUS_OK
        }
    };
    memcpy(response.getFeederId.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(GetFeederIdResponse));
}

void PhotonFeederProtocol::handleInitializeFeeder() {
    //Payload: <command id> <uuid:12>
    //Response: <feeder address> <ok>
    
    // Check uuid is correct, if not return a Wrong Feeder UUID error
    bool requestedUUIDMatchesMine = memcmp(command.initializeFeeder.uuid, _uuid, UUID_LENGTH) == 0;
    if (! requestedUUIDMatchesMine) {
        response = {
            .initializeFeeder = {
                .status = STATUS_WRONG_FEEDER_ID,
            },
        };
        memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

        transmitResponse(sizeof(InitializeFeederResponse));
        return;
    }
    
    // Mark this feeder as initialized
    _initialized = true;

    response = {
        .initializeFeeder = {
            .status = STATUS_OK,
        },
    };
    memcpy(response.initializeFeeder.uuid, _uuid, UUID_LENGTH);

    transmitResponse(sizeof(InitializeFeederResponse));
}

void PhotonFeederProtocol::handleGetVersion() {
    response = {
        .protocolVersion = {
            .status = STATUS_OK,
            .version = MAX_PROTOCOL_VERSION,
        },
    };

    transmitResponse(sizeof(GetProtocolVersionResponse));
}

void PhotonFeederProtocol::move(uint8_t distance, bool forward) {
    if (! guardInitialized()) {
        return;
    }

    PhotonFeeder::FeedResult result = _feeder->feedDistance(distance, forward);

    switch (result)
    {
    case PhotonFeeder::FeedResult::SUCCESS:
        response = {
            .moveFeed = {
                .status = STATUS_OK,
            },
        };

        transmitResponse(sizeof(MoveFeedResponse));
        break;
    
    case PhotonFeeder::FeedResult::INVALID_LENGTH:    // For Now Handle Invalid Length & Motor Fault The Same
    case PhotonFeeder::FeedResult::MOTOR_FAULT:
        response = {
            .moveFeed = {
                .status = STATUS_MOTOR_FAULT,
            },
        };
        transmitResponse(sizeof(MoveFeedResponse));
        break;

    case PhotonFeeder::FeedResult::UNKNOWN_ERROR:
        //TODO: Send The Appropriate Response
        break;

    default:
        break;
    }
}

void PhotonFeederProtocol::handleMoveFeedForward() {
    // Payload: <command id> <distance>
    // Response: <feeder address> <ok>
    move(command.move.distance, true);
}

void PhotonFeederProtocol::handleMoveFeedBackward() {
    // Payload: <command id> <distance>
    // Response: <feeder address> <ok>
    move(command.move.distance, false);
}

void PhotonFeederProtocol::handleGetFeederAddress() {
    //Payload: <command id> <uuid:12>
    //Response: <feeder address> <ok> <uuid:12>

    // Check For Feeder Match
    bool requestedUUIDMatchesMine = memcmp(command.initializeFeeder.uuid, _uuid, UUID_LENGTH) == 0;
    if (! requestedUUIDMatchesMine) {
        return;
    }

    response = {
        .getFeederAddress = {
            .status = STATUS_OK,
        },
    };

    transmitResponse(sizeof(GetFeederAddressResponse));
}

void PhotonFeederProtocol::transmitResponse(uint8_t payloadLength) {
    response.header = {
            .toAddress = PHOTON_NETWORK_CONTROLLER_ADDRESS,
            .fromAddress = _network->getLocalAddress(),
            .packetId = command.header.packetId,
            .payloadLength = payloadLength,
    };

    ModbusRTUChecksum crc;

    crc.add(response.header.toAddress);
    crc.add(response.header.fromAddress);
    crc.add(response.header.packetId);
    crc.add(response.header.payloadLength);

    for(uint8_t i = 0; i<response.header.payloadLength; i++){
        crc.add(responseBuffer[sizeof(PhotonPacketHeader) + i]);
    }

    response.header.checksum = (uint8_t)(crc & 0x0ff);

    size_t totalPacketLength = sizeof(PhotonPacketHeader) + response.header.payloadLength;
    _network->transmitPacket(responseBuffer, totalPacketLength);
}