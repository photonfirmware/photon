#ifndef _PHOTON_FEEDER_PROTOCOL_H
#define _PHOTON_FEEDER_PROTOCOL_H

#include "PhotonPacketHandler.h"
#include "Feeder.h"

#define UUID_LENGTH 12

class PhotonFeederProtocol : public PhotonPacketHandler {

    public:
        PhotonFeederProtocol(Feeder *feeder, const uint8_t *uuid, size_t uuid_length);
        void handle(PhotonNetworkLayer *instance, uint8_t *buffer, size_t buffer_length) override;
        bool isInitialized();
    
    private:

        Feeder *_feeder;
        uint8_t _uuid[UUID_LENGTH];
        bool _initialized;

        bool checkLength(uint8_t command_id, size_t command_payload_length);
        bool checkInitialized(PhotonNetworkLayer *instance);
        void handleGetFeederId(PhotonNetworkLayer *instance);
        void handleInitializeFeeder(PhotonNetworkLayer *instance, uint8_t *payload, size_t payload_length);
        void handleGetVersion(PhotonNetworkLayer *instance);
        void handleMoveFeedForward(PhotonNetworkLayer *instance, uint8_t *payload, size_t payload_length);
        void handleMoveFeedBackward(PhotonNetworkLayer *instance, uint8_t *payload, size_t payload_length);
        void handleGetFeederAddress(PhotonNetworkLayer *instance, uint8_t *payload, size_t payload_length);

        void move(PhotonNetworkLayer *instance, uint8_t distance, bool forwrd);
};

#endif //_PHOTON_FEEDER_PROTOCOL_H