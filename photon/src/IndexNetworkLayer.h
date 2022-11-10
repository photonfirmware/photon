#ifndef _INDEX_PROTOCOL_H
#define _INDEX_PROTOCOL_H

#include "define.h"

#include <cstddef>
#include <cstdint>
#include <IndexPacketHandler.h>
#include <FastCRC.h>
#include "Stream.h"

#include <rs485/rs485bus.hpp>
#include <rs485/packetizer.h>

#define INDEX_NETWORK_MAX_PDU 32
#define INDEX_PROTOCOL_CHECKSUM_LENGTH 2

#define INDEX_NETWORK_CONTROLLER_ADDRESS 0x00
#define INDEX_NETWORK_BROADCAST_ADDRESS 0xFF

class IndexNetworkLayer
{
public:
    IndexNetworkLayer(Packetizer* packetizer, RS485Bus<RS485_BUS_BUFFER_SIZE>* bus, uint8_t address, IndexPacketHandler* handler);

    virtual void setTimeoutPeriod(uint32_t timeout);
    virtual uint32_t getTimeoutPeriod();

    virtual void setLocalAddress(uint8_t address);
    virtual uint8_t getLocalAddress();

    virtual void tick();

    virtual bool transmitPacket(uint8_t destination_address, const uint8_t *buffer, size_t buffer_length);

private:
    FastCRC16 _CRC16;

    enum ProtocolState
    {
        AWAITING_ADDRESS,
        ADDRESS_RECEIVED,
        LENGTH_RECEIVED,
        PAYLOAD_RECEIVED
    };

    Packetizer* _packetizer;
    RS485Bus<RS485_BUS_BUFFER_SIZE>* _bus;
    bool _rs485_enable;
    uint8_t _buffer[RS485_BUS_BUFFER_SIZE];
    uint8_t _de_pin;
    uint8_t _re_pin;
    uint8_t _local_address;
    IndexPacketHandler* _handler;
    uint8_t _address;
    uint8_t _length;
    uint8_t _index;
    uint8_t _payload[INDEX_NETWORK_MAX_PDU];
    uint8_t _rx_checksum[INDEX_PROTOCOL_CHECKSUM_LENGTH];
    uint32_t _last_byte_time;
    uint32_t _timeout_period;

    void process(uint8_t *buffer, size_t buffer_length);
    void reset();
};

#endif //_INDEX_PROTOCOL_H