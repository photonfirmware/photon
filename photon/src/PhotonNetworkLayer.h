#ifndef _PHOTON_NETWORK_LAYER_H
#define _PHOTON_NETWORK_LAYER_H

#include "define.h"

#include "FeederFloor.h"
#include <cstddef>
#include <cstdint>
#include "PhotonPacketHandler.h"
#include "Stream.h"

#include <rs485/filters/filter_by_value.h>
#include <rs485/rs485bus.hpp>
#include <rs485/packetizer.h>
#include "rs485/protocols/checksums/modbus_rtu.h"

#define PHOTON_NETWORK_MAX_PDU 32
#define PHOTON_PROTOCOL_CHECKSUM_LENGTH 2

class PhotonNetworkLayer
{
public:
    PhotonNetworkLayer(
    RS485Bus<RS485_BUS_BUFFER_SIZE>* bus,
    Packetizer* packetizer,
    FilterByValue* addressFilter,
    FeederFloor* feederFloor);

    void setLocalAddress(uint8_t address);
    uint8_t getLocalAddress();

    bool getPacket(uint8_t* buffer, size_t maxBufferLength);

    bool transmitPacket(const uint8_t *buffer, size_t buffer_length);

private:
    RS485Bus<RS485_BUS_BUFFER_SIZE>* _bus;
    Packetizer* _packetizer;
    FilterByValue* _addressFilter;
    FeederFloor* _feederFloor;
    uint8_t _send_buffer[64];

    uint8_t _local_address;
};

#endif //_PHOTON_NETWORK_LAYER_H