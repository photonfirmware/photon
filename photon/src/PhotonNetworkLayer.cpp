#include "define.h"

#include "PhotonNetworkLayer.h"
#include "FeederFloor.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>


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
    FeederFloor* feederFloor) :
    _bus(bus),
    _packetizer(packetizer),
    _addressFilter(addressFilter),
    _feederFloor(feederFloor),
    _local_address(0xFF) {
    _local_address = _feederFloor->read_floor_address();

    _packetizer->setFilter(*_addressFilter);
    _addressFilter->preValues.allowAll();
    _addressFilter->postValues.allow(0xFF);

    // If floor address isn't set, _local_address is 0xFF and this function does nothing
    _addressFilter->postValues.allow(_local_address);
}

void PhotonNetworkLayer::setLocalAddress(uint8_t address) {
    if(_local_address != 0xff) {
        // The old address is no longer valid
        _addressFilter->postValues.reject(_local_address);
    }

    _local_address = address;
    _addressFilter->postValues.allow(_local_address);
}

uint8_t PhotonNetworkLayer::getLocalAddress() {
    return _local_address;
}

bool PhotonNetworkLayer::getPacket(uint8_t* buffer, size_t maxBufferLength) {
  // triggers if the packetizer detects that it has a packet
  if (! _packetizer->hasPacket()){
    return false;
  }

  size_t packet_length = min(_packetizer->packetLength(), maxBufferLength);
  // iterate through all bytes in RS485 object and plop them in the buffer
  for(size_t i = 0; i<packet_length; i++){
    buffer[i] = (*_bus)[i];
  }

  // Ideally the RS485 library handes delay
  delay(10);

  // clear the packet
  _packetizer->clearPacket();

  return true;
}

void PhotonNetworkLayer::clearPackets() {
  while(_packetizer->hasPacket()){
    _packetizer->clearPacket();
  }
}

bool PhotonNetworkLayer::transmitPacket(const uint8_t *buffer, size_t buffer_length) {
    _packetizer->writePacket(buffer, buffer_length);

    // TODO Handle write packet status result

    return true;
}