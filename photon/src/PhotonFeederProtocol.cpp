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

typedef struct {
    FeederCommand command_id;
    size_t min_payload_length;
    size_t max_payload_length;
} protocol_command_t;

// Note that this array MUST be sorted as the function handle makes that assumption for efficiency
static protocol_command_t commands[] = {
    {GET_FEEDER_ID, 0, 0},
    {INITIALIZE_FEEDER, 12, 12},
    {GET_VERSION, 0, 0},
    {MOVE_FEED_FORWARD, 1, 1},
    {MOVE_FEED_BACKWARD, 1, 1},
    {GET_FEEDER_ADDRESS, 12, 12}
};
static const size_t num_commands = sizeof(commands) / sizeof(protocol_command_t);

static const uint8_t zero_uuid[UUID_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define GET_FEEDER_ID_RESPONSE_LENGTH (2 + UUID_LENGTH)
#define INITIALIZE_FEEDER_RESPONSE_LENGTH   2
#define GET_VERSION_RESPONSE_LENGTH 3
#define GET_FEEDER_ADDRESS_RESPONSE_LENGTH (2 + UUID_LENGTH)

#define MOVE_FEED_RESPONSE_LENGTH 2

#define WRONG_FEEDER_RESPONSE_LENGTH (2 + UUID_LENGTH)

#define UNINITIALIZED_FEEDER_RESPONSE_LENGTH (2 + UUID_LENGTH)

PhotonFeederProtocol::PhotonFeederProtocol(
    PhotonFeeder *feeder,
    PhotonNetworkLayer* network,
    const uint8_t *uuid, size_t uuid_length) : _feeder(feeder), _network(network), _initialized(false) {
    memset(_uuid, 0, UUID_LENGTH);
    memcpy(_uuid, uuid, (uuid_length < UUID_LENGTH) ? uuid_length : UUID_LENGTH);
}

void PhotonFeederProtocol::tick() {
    bool hasPacket = _network->getPacket(packetCommandBuffer.buffer, sizeof(packetCommandBuffer.buffer));

    if(! hasPacket) {
        return;
    }

    switch(packetCommandBuffer.command.commandId) {
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

bool PhotonFeederProtocol::checkInitialized() {
    // Checks if the feeder is initialized and returns an error if it hasn't been
    // Response: <feeder address> 0x03 <uuid:12>

    if (!_initialized) {
        uint8_t response[UNINITIALIZED_FEEDER_RESPONSE_LENGTH];
        response[0] = _network->getLocalAddress();
        response[1] = STATUS_UNINITIALIZED_FEEDER;
        memcpy(&response[2], _uuid, UUID_LENGTH);
        _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, UNINITIALIZED_FEEDER_RESPONSE_LENGTH);
    }

    return _initialized;
}


void PhotonFeederProtocol::handleGetFeederId() {
    // Payload: <command id>
    // Response: <feeder address> <ok> <uuid:12>

    uint8_t response[GET_FEEDER_ID_RESPONSE_LENGTH];
    response[0] = _network->getLocalAddress();
    response[1] = STATUS_OK;
    memcpy(&response[2], _uuid, UUID_LENGTH);

    _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, GET_FEEDER_ID_RESPONSE_LENGTH);
}

void PhotonFeederProtocol::handleInitializeFeeder() {
    //Payload: <command id> <uuid:12>
    //Response: <feeder address> <ok>
    
    // Check uuid is correct or all zeros, if not return a Wrong Feeder UUID error
    if (memcmp(packetCommandBuffer.command.initializeFeeder.uuid, zero_uuid, UUID_LENGTH) != 0 &&
        memcmp(packetCommandBuffer.command.initializeFeeder.uuid, _uuid, UUID_LENGTH) != 0) {
        uint8_t response[WRONG_FEEDER_RESPONSE_LENGTH];
        response[0] = _network->getLocalAddress();
        response[1] = STATUS_WRONG_FEEDER_ID;
        memcpy(&response[2], _uuid, UUID_LENGTH);
        _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, WRONG_FEEDER_RESPONSE_LENGTH);
        return;
    }
    
    // Do Response
    _initialized = true;

    // Start To Setup Response
    uint8_t response[INITIALIZE_FEEDER_RESPONSE_LENGTH];
    response[0] = _network->getLocalAddress();
    response[1] = STATUS_OK;
    _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, INITIALIZE_FEEDER_RESPONSE_LENGTH);
}

void PhotonFeederProtocol::handleGetVersion() {
    uint8_t response[GET_VERSION_RESPONSE_LENGTH];

    // Build the response
    response[0] = _network->getLocalAddress();
    response[1] = STATUS_OK;
    response[2] = MAX_PROTOCOL_VERSION;

    // Transmit The Packet To The Central
    _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, GET_VERSION_RESPONSE_LENGTH);
}

void PhotonFeederProtocol::move(uint8_t distance, bool forward) {

    if (!checkInitialized()) {
        return;
    }

    PhotonFeeder::FeedResult result = _feeder->feedDistance(distance, forward);

    switch (result)
    {
    case PhotonFeeder::FeedResult::SUCCESS: 
    {
        uint8_t response[MOVE_FEED_RESPONSE_LENGTH];
        response[0] = _network->getLocalAddress();
        response[1] = STATUS_OK;
        _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, MOVE_FEED_RESPONSE_LENGTH);
    }
        break;
    
    case PhotonFeeder::FeedResult::INVALID_LENGTH:    // For Now Handle Invalid Length & Motor Fault The Same
    case PhotonFeeder::FeedResult::MOTOR_FAULT:
    {
        uint8_t response[MOVE_FEED_RESPONSE_LENGTH];
        response[0] = _network->getLocalAddress();
        response[1] = STATUS_MOTOR_FAULT;
        _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, MOVE_FEED_RESPONSE_LENGTH);
    }
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
    move(packetCommandBuffer.command.move.distance, true);
}

void PhotonFeederProtocol::handleMoveFeedBackward() {
    // Payload: <command id> <distance>
    // Response: <feeder address> <ok>
    move(packetCommandBuffer.command.move.distance, false);
}

void PhotonFeederProtocol::handleGetFeederAddress() {
    //Payload: <command id> <uuid:12>
    //Response: <feeder address> <ok> <uuid:12>

    // Check For Feeder Match
    if (memcmp(packetCommandBuffer.command.getFeederAddress.uuid, _uuid, UUID_LENGTH) != 0) {
        return;
    }

    // Return The Address
    uint8_t response[GET_FEEDER_ADDRESS_RESPONSE_LENGTH];
    response[0] = _network->getLocalAddress();
    response[1] = STATUS_OK;
    memcpy(&response[2], _uuid, UUID_LENGTH);

    _network->transmitPacket(PHOTON_NETWORK_CONTROLLER_ADDRESS, response, GET_FEEDER_ADDRESS_RESPONSE_LENGTH);
}