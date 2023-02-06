#pragma once

#include <OneWire.h>

class FeederFloor {
  public:
    explicit FeederFloor(OneWire* oneWire);

    uint8_t read_floor_address();
    bool write_floor_address(uint8_t address);

  private: 
    OneWire* _oneWire;
};