#ifndef _PHOTON_FEEDER_PROTOCOL_H
#define _PHOTON_FEEDER_PROTOCOL_H

#include "FeederFloor.h"
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
    uint8_t crc;
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

struct PACKED VendorOptionsCommand {
    uint8_t options[VENDOR_SPECIFIC_OPTIONS_LENGTH];
};

struct PACKED ProgramFeederFloorAddressCommand {
    uint8_t uuid[UUID_LENGTH];
    uint8_t address;
};

struct PACKED IdentifyFeederCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED PhotonCommand {
    PhotonPacketHeader header;
    uint8_t commandId;
    union {
        MoveCommand move;
        GetFeederAddressCommand getFeederAddress;
        InitializeFeederCommand initializeFeeder;
        VendorOptionsCommand vendorOptions;
        ProgramFeederFloorAddressCommand programFeederFloorAddress;
        IdentifyFeederCommand identifyFeeder;
    };
};

struct PACKED GetFeederIdResponse {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED InitializeFeederResponse {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED GetProtocolVersionResponse {
    uint8_t version;
};

struct PACKED FeedDistanceResponse {
    uint16_t expectedFeedTime;
};

struct PACKED PhotonResponse {
    PhotonPacketHeader header;
    uint8_t status;
    union {
        GetFeederIdResponse getFeederId;
        InitializeFeederResponse initializeFeeder;
        GetProtocolVersionResponse protocolVersion;
        FeedDistanceResponse expectedTimeToFeed;
    };
};

class PhotonFeederProtocol {

    public:
        PhotonFeederProtocol(
            PhotonFeeder *feeder,
            FeederFloor *feederFloor,
            PhotonNetworkLayer* network,
            const uint8_t *uuid,
            size_t uuid_length
        );
        void tick();
    
    private:
        PhotonFeeder* _feeder;
        FeederFloor* _feederFloor;
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

        // Unicast
        bool guardInitialized();
        void handleGetFeederId();
        void handleInitializeFeeder();
        void handleGetVersion();
        void handleMoveFeedForward();
        void handleMoveFeedBackward();
        void handleMoveFeedStatus();
        void handleVendorOptions();

        // Broadcast
        void handleGetFeederAddress();
        void handleIdentifyFeeder();
        void handleProgramFeederFloor();

        void move(uint8_t distance, bool forward);
        bool isInitialized();

        void transmitResponse(uint8_t responseSize = 0);
};

#endif //_PHOTON_FEEDER_PROTOCOL_H