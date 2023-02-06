#ifndef _PHOTON_NETWORK_LAYER_H
#define _PHOTON_NETWORK_LAYER_H

#include "define.h"

#include <cstddef>
#include <cstdint>
#include "PhotonPacketHandler.h"
//#include <FastCRC.h>
#include "Stream.h"

#include <rs485/rs485bus.hpp>
#include <rs485/packetizer.h>
#include "rs485/protocols/checksums/modbus_rtu.h"

#define PHOTON_NETWORK_MAX_PDU 32
#define PHOTON_PROTOCOL_CHECKSUM_LENGTH 2

#define PHOTON_NETWORK_CONTROLLER_ADDRESS 0x00
#define PHOTON_NETWORK_BROADCAST_ADDRESS 0xFF

class PhotonNetworkLayer
{
public:
    PhotonNetworkLayer(Packetizer* packetizer, RS485Bus<RS485_BUS_BUFFER_SIZE>* bus, uint8_t address, PhotonPacketHandler* handler);

    virtual void setTimeoutPeriod(uint32_t timeout);
    virtual uint32_t getTimeoutPeriod();

    virtual void setLocalAddress(uint8_t address);
    virtual uint8_t getLocalAddress();

    virtual uint8_t tick();

    virtual bool transmitPacket(uint8_t destination_address, const uint8_t *buffer, size_t buffer_length);

private:
//    FastCRC16 _CRC16;

    enum ProtocolState
    {
        AWAITING_ADDRESS,
        ADDRESS_RECEIVED,
        LENGTH_RECEIVED,
        PAYLOAD_RECEIVED
    };

    Packetizer* _packetizer;
    RS485Bus<RS485_BUS_BUFFER_SIZE>* _bus;
    uint8_t _local_address;
    PhotonPacketHandler* _handler;
    uint8_t _address;
    uint8_t _length;
    uint8_t _send_buffer[64];
    uint8_t _payload[PHOTON_NETWORK_MAX_PDU];
    uint8_t _rx_checksum[PHOTON_PROTOCOL_CHECKSUM_LENGTH];
    uint32_t _last_byte_time;
    uint32_t _timeout_period;

    void process(uint8_t *buffer, size_t buffer_length);
    void reset();
};

#endif //_PHOTON_NETWORK_LAYER_H