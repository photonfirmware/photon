#ifndef _PHOTON_FEEDER_PROTOCOL_H
#define _PHOTON_FEEDER_PROTOCOL_H

#include "PhotonFeeder.h"
#include "PhotonNetworkLayer.h"

#define UUID_LENGTH 12

#define PHOTON_NETWORK_CONTROLLER_ADDRESS 0x00
#define PHOTON_NETWORK_BROADCAST_ADDRESS 0xFF

#define PACKED __attribute__ ((packed))

struct PACKED PhotonPacketHeader {
    uint8_t toAddress;
    uint8_t fromAddress;
    uint8_t packetId;
    uint8_t payloadLength;
    uint8_t checksum;
};

struct PACKED MoveCommand {
    uint8_t distance;
};

struct PACKED GetFeederAddressCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED InitializeFeederCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED PhotonCommand {
    PhotonPacketHeader header;
    uint8_t commandId;
    union {
        MoveCommand move;
        GetFeederAddressCommand getFeederAddress;
        InitializeFeederCommand initializeFeeder;
    };
};

struct GetFeederIdResponse {
    uint8_t status;
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED InitializeFeederResponse {
    uint8_t status;
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED GetProtocolVersionResponse {
    uint8_t status;
    uint8_t version;
};

struct PACKED MoveFeedResponse {
    uint8_t status;
};

struct PACKED GetFeederAddressResponse {
    uint8_t status;
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED PhotonResponse {
    PhotonPacketHeader header;
    union {
        GetFeederIdResponse getFeederId;
        InitializeFeederResponse initializeFeeder;
        GetProtocolVersionResponse protocolVersion;
        MoveFeedResponse moveFeed;
        GetFeederAddressResponse getFeederAddress;
    };
};

class PhotonFeederProtocol {

    public:
        PhotonFeederProtocol(
            PhotonFeeder *feeder,
            PhotonNetworkLayer* network,
            const uint8_t *uuid,
            size_t uuid_length
        );
        void tick();
    
    private:
        PhotonFeeder* _feeder;
        PhotonNetworkLayer* _network;
        uint8_t _uuid[UUID_LENGTH];
        bool _initialized;

        union {
            PhotonCommand command;
            uint8_t commandBuffer[sizeof(PhotonCommand)];
        };

        union {
            PhotonResponse response;
            uint8_t responseBuffer[sizeof(PhotonResponse)];
        };

        bool guardInitialized();
        void handleGetFeederId();
        void handleInitializeFeeder();
        void handleGetVersion();
        void handleMoveFeedForward();
        void handleMoveFeedBackward();
        void handleMoveFeedStatus(); 
        void handleGetFeederAddress();

        void move(uint8_t distance, bool forwrd);
        bool isInitialized();

        void transmitResponse(uint8_t payloadLength);
};

#endif //_PHOTON_FEEDER_PROTOCOL_H