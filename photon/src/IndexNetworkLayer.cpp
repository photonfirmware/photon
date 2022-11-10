#include "define.h"

#include "IndexNetworkLayer.h"
#include "Arduino.h"
#include <cstring>
#include <cstdio>

#include <rs485/rs485bus.hpp>
#include <rs485/packetizer.h>

#ifndef NATIVE
#include "util.h"
#endif

#define INDEX_PROTOCOL_DEFAULT_TIMEOUT_MS 100
#define INDEX_INCOMING_BUFFER_SIZE 16
#define RS485_CONTROL_DELAY 10

IndexNetworkLayer::IndexNetworkLayer(Packetizer* packetizer, RS485Bus<RS485_BUS_BUFFER_SIZE>* bus, uint8_t address, IndexPacketHandler* handler) :  _packetizer(packetizer), _rs485_enable(false), _bus(bus), _local_address(address), _handler(handler), _timeout_period(INDEX_PROTOCOL_DEFAULT_TIMEOUT_MS) {
    reset();
}

void IndexNetworkLayer::setTimeoutPeriod(uint32_t timeout) {
    _timeout_period = timeout;
}

uint32_t IndexNetworkLayer::getTimeoutPeriod() {
    return _timeout_period;
}

void IndexNetworkLayer::setLocalAddress(uint8_t address) {
    _local_address = address;
}

uint8_t IndexNetworkLayer::getLocalAddress() {
    return _local_address;
}

void IndexNetworkLayer::tick() {

    // triggers if the packetizer detects that it has a packet
    if(_packetizer->hasPacket()){
        // find packet length
        uint8_t packet_length = _packetizer->packetLength();
        // make buffer of that length
        uint8_t buffer[packet_length];

        // iterate through all bytes in RS485 object and plop them in the buffer
        for(int i = 0; i<packet_length; i++){
            buffer[i] = (*_bus)[i];
        }

        // process the buffer
        //process(buffer, packet_length);
        _handler->handle(this, _buffer, _length);

        // clear the packet
        _packetizer->clearPacket();

    }//0201011050


}

bool IndexNetworkLayer::transmitPacket(uint8_t destination_address, const uint8_t *buffer, size_t buffer_length) {

    // Do some very basic integrity checks to make sure the call was valid
    if (NULL == buffer || buffer_length > INDEX_NETWORK_MAX_PDU || buffer_length > UINT8_MAX) {
        return false;
    }

    uint8_t length = buffer_length;
    uint8_t crc_array[INDEX_PROTOCOL_CHECKSUM_LENGTH];
    uint16_t crc = _CRC16.modbus(&destination_address, 1);
    crc = _CRC16.modbus_upd(&length, 1);
    crc = _CRC16.modbus_upd(buffer, buffer_length);
    crc = htons(crc);

    crc_array[0] = (uint8_t)((crc >> 8) & 0x0ff);
    crc_array[1] = (uint8_t)(crc & 0x0ff);

    // // Transmit The Address
    // _stream->write(&destination_address, 1);
    
    // // Transmit The Length
    // _stream->write(&length, 1);

    // // Transmit The Data
    // _stream->write(buffer, buffer_length);

    // // Transmit CRC
    // _stream->write(crc_array, INDEX_PROTOCOL_CHECKSUM_LENGTH);

    // first, find the length of the packet we're sending
    uint8_t packet_buffer_length = buffer_length + 4;

    // then, make a buffer that size
    uint8_t packet_buffer[packet_buffer_length];

    // now drop in the destination address and length
    packet_buffer[0] = destination_address;
    packet_buffer[1] = length;

    // drop in the data buffer
    for(int i = 0; i < length; i++){
        packet_buffer[i + 2] = buffer[i];
    }

    // add crc bytes
    packet_buffer[packet_buffer_length - 2] = crc_array[0];
    packet_buffer[packet_buffer_length - 1] = crc_array[1];

    _packetizer->writePacket(packet_buffer, packet_buffer_length);

    return true;
}

void IndexNetworkLayer::reset() {
    // Reset The State Machine
    _address = 0;
    _length = 0;
    _index = 0;
    memset(_payload, 0, INDEX_NETWORK_MAX_PDU);
    memset(_rx_checksum, 0, INDEX_PROTOCOL_CHECKSUM_LENGTH);
    _last_byte_time = 0;

}
