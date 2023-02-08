#ifndef _PHOTON_FEEDER_PROTOCOL_H
#define _PHOTON_FEEDER_PROTOCOL_H

#include "PhotonFeeder.h"
#include "PhotonNetworkLayer.h"

#define UUID_LENGTH 12

struct __attribute__ ((packed)) PhotonPacketHeader {
    uint8_t fromAddress;
    uint8_t toAddress;
    uint8_t packetId;
    uint8_t packetLength;
    uint8_t checksum;
};

struct __attribute__ ((packed)) MoveCommand {
    uint8_t distance;
};

struct __attribute__ ((packed)) GetFeederAddressCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct __attribute__ ((packed)) InitializeFeederCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct __attribute__ ((packed)) PhotonCommand {
    PhotonPacketHeader header;
    uint8_t commandId;
    union {
        MoveCommand move;
        GetFeederAddressCommand getFeederAddress;
        InitializeFeederCommand initializeFeeder;
    };
};

struct __attribute__ ((packed)) PhotonResponse {
    PhotonPacketHeader header;
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
            uint8_t buffer[sizeof(PhotonCommand)];
        } packetCommandBuffer;

        union {
            PhotonResponse response;
            uint8_t buffer[sizeof(PhotonResponse)];
        } packetResponseBuffer;

        bool checkInitialized();
        void handleGetFeederId();
        void handleInitializeFeeder();
        void handleGetVersion();
        void handleMoveFeedForward();
        void handleMoveFeedBackward();
        void handleGetFeederAddress();

        void move(uint8_t distance, bool forwrd);
        bool isInitialized();
};

#endif //_PHOTON_FEEDER_PROTOCOL_H