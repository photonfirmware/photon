#include "define.h"

#include "PhotonNetworkLayer.h"
#include "FeederFloor.h"
#include <Arduino.h>
#include <cstring>
#include <cstdio>


#ifndef NATIVE
#include "util.h"
#endif

#define PHOTON_PROTOCOL_DEFAULT_TIMEOUT_MS 100
#define PHOTON_INCOMING_BUFFER_SIZE 16
#define RS485_CONTROL_DELAY 10

PhotonNetworkLayer::PhotonNetworkLayer(
    RS485Bus<RS485_BUS_BUFFER_SIZE>* bus,
    Packetizer* packetizer,
    FilterByValue* addressFilter,
    FeederFloor* feederFloor,
    PhotonPacketHandler* handler) :
    _bus(bus),
    _packetizer(packetizer),
    _addressFilter(addressFilter),
    _feederFloor(feederFloor),
    _handler(handler),
    _local_address(0xFF) {
    _local_address = _feederFloor->read_floor_address();

    _packetizer->setFilter(*_addressFilter);
    _addressFilter->postValues.allow(0xFF);

    // If floor address isn't set, _local_address is 0xFF and this function does nothing
    _addressFilter->postValues.allow(_local_address);
}

void PhotonNetworkLayer::setLocalAddress(uint8_t address) {
    _local_address = address;
}

uint8_t PhotonNetworkLayer::getLocalAddress() {
    return _local_address;
}

void PhotonNetworkLayer::tick() {
  // triggers if the packetizer detects that it has a packet
  if (! _packetizer->hasPacket()){
    return;
  }

  uint8_t packet_length = _packetizer->packetLength();

  if(packet_length == 0){
    return;
  }

  // make buffer of that length
  uint8_t buffer[packet_length];

  // iterate through all bytes in RS485 object and plop them in the buffer
  for(int i = 0; i<packet_length; i++){
    buffer[i] = (*_bus)[i];
  }

  delay(10);

  // handle buffer
  _handler->handle(this, buffer, packet_length);

  // clear the packet
  _packetizer->clearPacket();
}

bool PhotonNetworkLayer::transmitPacket(uint8_t destination_address, const uint8_t *buffer, size_t buffer_length) {

    // Do some very basic integrity checks to make sure the call was valid
    if (NULL == buffer || buffer_length > PHOTON_NETWORK_MAX_PDU || buffer_length > UINT8_MAX) {
        return false;
    }

    uint8_t crc_array[PHOTON_PROTOCOL_CHECKSUM_LENGTH];
    ModbusRTUChecksum crc;

    crc.add(destination_address);
    crc.add(buffer_length);

    for(int i = 0; i<buffer_length;i++){
        crc.add(buffer[i]);
    }

    uint16_t checksum = crc.getChecksum();

    crc_array[0] = (uint8_t)(crc & 0x0ff);
    crc_array[1] = (uint8_t)((crc >> 8) & 0x0ff);

    // first, find the length of the packet we're sending
    uint8_t send_buffer_length = buffer_length + 4;

    // now drop in the destination address and length
    _send_buffer[0] = destination_address;
    _send_buffer[1] = buffer_length;

    // drop in the data buffer
    for(int i = 0; i < buffer_length; i++){
        _send_buffer[i + 2] = buffer[i];
    }

    // add crc bytes
    _send_buffer[send_buffer_length - 2] = crc_array[0];
    _send_buffer[send_buffer_length - 1] = crc_array[1];

    _packetizer->writePacket(_send_buffer, send_buffer_length);

    return true;
}